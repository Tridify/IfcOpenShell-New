/********************************************************************************
*                                                                              *
* This file is part of IfcOpenShell.                                           *
*                                                                              *
* IfcOpenShell is free software: you can redistribute it and/or modify         *
* it under the terms of the Lesser GNU General Public License as published by  *
* the Free Software Foundation, either version 3.0 of the License, or          *
* (at your option) any later version.                                          *
*                                                                              *
* IfcOpenShell is distributed in the hope that it will be useful,              *
* but WITHOUT ANY WARRANTY; without even the implied warranty of               *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 *
* Lesser GNU General Public License for more details.                          *
*                                                                              *
* You should have received a copy of the Lesser GNU General Public License     *
* along with this program. If not, see <http://www.gnu.org/licenses/>.         *
*                                                                              *
********************************************************************************/

#include "JsonSerializer.h"

#include "json.hpp"
#include "../ifcparse/IfcSIPrefix.h"
#include "../ifcgeom/IfcGeom.h"
#include "../ifcparse/utils.h"

#include <map>
#include <algorithm>
#include <utility>

using namespace IfcSchema;
using json = nlohmann::json;

// The ifc json version written to the exported json file
#define IFC_JSON_VERSION "1.0"

// Utils
namespace {
    std::vector<std::string> filter_empty_strings(const std::vector<std::string>& source) {
        std::vector<std::string> filtered_items;

        for (const std::string& item : source) {
            if (item.length() > 0) {
                filtered_items.push_back(item);
            }
        }

        return filtered_items;
    }
}

namespace {
    std::map<std::string, std::string> argument_name_map;

    // Format an IFC attribute and maybe returns as string. Only literal scalar values are converted.
    // Things like entity instances and lists are omitted.
    boost::optional<std::string> format_attribute(const Argument* argument, IfcUtil::ArgumentType argument_type, const std::string& argument_name) {
        boost::optional<std::string> value;

        // Hard-code lat-lon as it represents an array of integers best emitted as a single decimal
        if (argument_name == "IfcSite.RefLatitude" || argument_name == "IfcSite.RefLongitude") {
            std::vector<int> angle = *argument;
            double deg;

            if (angle.size() >= 3) {
                deg = angle[0] + angle[1] / 60. + angle[2] / 3600.;
                int precision = 8;

                if (angle.size() == 4) {
                    deg += angle[3] / (1000000. * 3600.);
                    precision = 14;
                }

                std::stringstream stream;
                stream << std::setprecision(precision) << deg;
                value = stream.str();
            }

            return value;
        }

        switch(argument_type) {
            case IfcUtil::Argument_BOOL: {
                const bool b = *argument;
                value = b ? "true" : "false";
                break;
            }
            case IfcUtil::Argument_DOUBLE: {
                const double d = *argument;
                std::stringstream stream;
                stream << d;
                value = stream.str();
                break;
            }
            case IfcUtil::Argument_STRING:
            case IfcUtil::Argument_ENUMERATION: {
                value = static_cast<std::string>(*argument);
                break;
            }
            case IfcUtil::Argument_INT: {
                const int v = *argument;
                std::stringstream stream;
                stream << v;
                value = stream.str();
                break;
            }
            case IfcUtil::Argument_ENTITY_INSTANCE: {
                IfcUtil::IfcBaseClass* e = *argument;

                if (Type::IsSimple(e->type())) {
                    IfcUtil::IfcBaseType* f = (IfcUtil::IfcBaseType*) e;
                    value = format_attribute(f->getArgument(0), f->getArgumentType(0), argument_name);
                }
                else if (e->is(IfcSchema::Type::IfcSIUnit) || e->is(IfcSchema::Type::IfcConversionBasedUnit)) {
                    // Some string concatenation to have a unit name as a XML attribute.
                    std::string unit_name;

                    if (e->is(IfcSchema::Type::IfcSIUnit)) {
                        IfcSchema::IfcSIUnit* unit = (IfcSchema::IfcSIUnit*) e;
                        unit_name = IfcSchema::IfcSIUnitName::ToString(unit->Name());
                        
                        if (unit->hasPrefix()) {
                            unit_name = IfcSchema::IfcSIPrefix::ToString(unit->Prefix()) + unit_name;
                        }
                    }
                    else {
                        IfcSchema::IfcConversionBasedUnit* unit = (IfcSchema::IfcConversionBasedUnit*) e;
                        unit_name = unit->Name();
                    }

                    value = unit_name;
                }
                else if (e->is(IfcSchema::Type::IfcLocalPlacement)) {
                    IfcSchema::IfcLocalPlacement* placement = e->as<IfcSchema::IfcLocalPlacement>();
                    gp_Trsf transform;
                    IfcGeom::Kernel kernel;

                    if (kernel.convert(placement, transform)) {
                        std::stringstream stream;

                        for (int i = 1; i < 5; ++i) {
                            for (int j = 1; j < 4; ++j) {
                                const double transform_value = transform.Value(j, i);
                                stream << transform_value << " ";
                            }

                            stream << ((i == 4) ? "1" : "0 ");
                        }
                        value = stream.str();
                    }				
                }
                break;
            }
            default:
                break;
        }

        return value;
    }

    // Formats IFC entity instance, adds properties to the referenced jsonObject
    void format_entity_instance(IfcUtil::IfcBaseEntity* instance, json::reference jsonObject, bool as_link = false) {
        if (!jsonObject.is_object()) {
            Logger::Error("Json reference must be an object!", instance->entity);
            return;
        }
        
        const unsigned n = instance->getArgumentCount();

        for (unsigned i = 0; i < n; ++i) {
            try {
                instance->getArgument(i);
            }
            catch (const std::exception&) {
                Logger::Error("Expected " + std::to_string(n) + " attributes for:", instance->entity);
                break;
            }

            const Argument* argument = instance->getArgument(i);

            if (argument->isNull()) continue;

            std::string argument_name = instance->getArgumentName(i);
            std::map<std::string, std::string>::const_iterator argument_name_it;
            argument_name_it = argument_name_map.find(argument_name);

            if (argument_name_it != argument_name_map.end()) {
                argument_name = argument_name_it->second;
            }

            const IfcUtil::ArgumentType argument_type = instance->getArgumentType(i);

            const std::string qualified_name = IfcSchema::Type::ToString(instance->type()) + "." + argument_name;
            boost::optional<std::string> value;

            try {
                value = format_attribute(argument, argument_type, qualified_name);
            }
            catch (const std::exception& e) {
                Logger::Error(e);
            }
            catch (const Standard_ConstructionError& e) {
                Logger::Error(e.GetMessageString(), instance->entity);
            }

            if (value) {
                if (as_link) {
                    if (argument_name == "id") {
                        jsonObject["@{http://www.w3.org/1999/xlink}href"] = std::string("#") + *value;
                    }
                }
                else {
                    std::stringstream stream;
                    stream << "@" << argument_name;
                    jsonObject[stream.str()] = *value;
                }
            }
        }
    }

    std::string qualify_unrooted_instance(IfcUtil::IfcBaseClass* inst) {
        return IfcSchema::Type::ToString(inst->type()) + "_" + std::to_string(inst->entity->id());
    }

    // Initializes an array in a jsonObject with jsonKey, creates an empty object to that array and returns the reference
    json::reference getEmptyObjectReferenceInArray(const std::string& jsonKey, json::reference jsonObject) {
        // Get a json reference to the target jsonKey
        json::reference arrayReference = jsonObject[jsonKey];

        // Initialize the array if not done previously
        if (arrayReference.is_null()) {
            arrayReference = json::array();
        }

        // Add empty object to array and return its reference
        arrayReference += json::object();
        return arrayReference.back();
    }

    // A function to be called recursively. Template specialization is used 
    // to descend into decomposition, containment and property relationships.
    template <typename A>
    void descend(A* instance, json::reference jsonObject) {
        if (!jsonObject.is_object() && !jsonObject.is_null()) {
            Logger::Error("Json reference must be an object or null!", instance->entity);
            return;
        }

        // If the instance is an IfcObjectDefinition -> can have children -> descend deeper
        if (instance->is(IfcSchema::Type::IfcObjectDefinition)) {
            // Use the created empty json object as the reference
            descend(instance->template as<IfcSchema::IfcObjectDefinition>(), jsonObject);
        }
        // Otherwise just format the instance
        else {
            // Initialize a container array for the IFC type if not created yet
            // add an empty object to that array and get the reference
            json::reference targetObject = instance->is(Type::IfcProject) ?
                jsonObject["IfcProject"] = json::object() :
                getEmptyObjectReferenceInArray(Type::ToString(instance->type()), jsonObject);

            // Add entity instance properties to the created empty json object
            format_entity_instance(instance, targetObject);
        }
    }

    // Returns related entity instances using IFC's objectified relationship model.
    // The second and third argument require a member function pointer.
    template <typename T, typename U, typename V, typename F, typename G>
    typename V::list::ptr get_related(T* t, F f, G g) {
        typename U::list::ptr li = (*t.*f)()->template as<U>();
        typename V::list::ptr acc(new typename V::list);

        for (typename U::list::it it = li->begin(); it != li->end(); ++it) {
            U* u = *it;
            acc->push((*u.*g)()->template as<V>());
        }

        return acc;
    }

    // Descends into the tree by recursion into IfcRelContainedInSpatialStructure,
    // IfcRelDecomposes, IfcRelDefinesByType, IfcRelDefinesByProperties relations.
    template <>
    void descend(IfcObjectDefinition* product, json::reference jsonObject) {
        if (!jsonObject.is_object() && !jsonObject.is_null()) {
            Logger::Error("Json reference must be an object or null!", product->entity);
            return;
        }

        // Initialize a container array for the IFC type if not created yet
        // add an empty object to that array and get the reference
        // There can be only one IfcProject so do not create an array of it
        json::reference targetObject = product->is(Type::IfcProject) ?
            jsonObject["IfcProject"] = json::object() :
            getEmptyObjectReferenceInArray(Type::ToString(product->type()), jsonObject);

        // Add entity instance properties to json object
        format_entity_instance(product, targetObject);

        // Handle IfcSpatialStructureElement mapping
        if (product->is(Type::IfcSpatialStructureElement)) {
            IfcSpatialStructureElement* structure = (IfcSpatialStructureElement*) product;

            IfcObjectDefinition::list::ptr elements = get_related
                <IfcSpatialStructureElement, IfcRelContainedInSpatialStructure, IfcObjectDefinition>
                (structure, &IfcSpatialStructureElement::ContainsElements, &IfcRelContainedInSpatialStructure::RelatedElements);

            for (IfcObjectDefinition::list::it it = elements->begin(); it != elements->end(); ++it) {
                descend(*it, targetObject);
            }
        }

        // Handle IfcElement mapping
        if (product->is(Type::IfcElement)) {
            IfcElement* element = static_cast<IfcElement*>(product);
            IfcOpeningElement::list::ptr openings = get_related<IfcElement, IfcRelVoidsElement, IfcOpeningElement>(
                element, &IfcElement::HasOpenings, &IfcRelVoidsElement::RelatedOpeningElement
            );

            for (IfcOpeningElement::list::it it = openings->begin(); it != openings->end(); ++it) {
                descend(*it, targetObject);
            }
        }

        // Handle structure mapping
#ifndef USE_IFC4
        IfcObjectDefinition::list::ptr structures = get_related<IfcObjectDefinition, IfcRelDecomposes, IfcObjectDefinition>(
            product, &IfcObjectDefinition::IsDecomposedBy, &IfcRelDecomposes::RelatedObjects
        );
#else
        IfcObjectDefinition::list::ptr structures = get_related<IfcObjectDefinition, IfcRelAggregates, IfcObjectDefinition>(
            product, &IfcProduct::IsDecomposedBy, &IfcRelAggregates::RelatedObjects
        );
#endif

        for (IfcObjectDefinition::list::it it = structures->begin(); it != structures->end(); ++it) {
            IfcObjectDefinition* ob = *it;
            descend(ob, targetObject);
        }
        
        // Handle IfcTypeObject mapping
        if (product->is(Type::IfcTypeObject)) {
            IfcTypeObject* type_object = product->as<IfcTypeObject>();
            
            if (type_object->hasHasPropertySets()) {
                IfcPropertySetDefinition::list::ptr property_sets = type_object->HasPropertySets();

                for (IfcPropertySetDefinition::list::it jt = property_sets->begin(); jt != property_sets->end(); ++jt) {
                    IfcPropertySetDefinition* pset = *jt;

                    if (pset->is(Type::IfcPropertySet)) {
                        json::reference propertyObject = getEmptyObjectReferenceInArray("IfcPropertySet", targetObject);

                        format_entity_instance(pset, propertyObject, true);
                    }
                }
            }
        }

        // Handle IfcObject mapping
        if (product->is(IfcSchema::Type::IfcObject)) {
            IfcSchema::IfcObject* object = product->as<IfcSchema::IfcObject>();

            IfcPropertySetDefinition::list::ptr property_sets = get_related<IfcObject, IfcRelDefinesByProperties, IfcPropertySetDefinition>(
                object, &IfcObject::IsDefinedBy, &IfcRelDefinesByProperties::RelatingPropertyDefinition
            );

            for (IfcPropertySetDefinition::list::it it = property_sets->begin(); it != property_sets->end(); ++it) {
                IfcPropertySetDefinition* pset = *it;

                json::reference to = getEmptyObjectReferenceInArray(Type::ToString(pset->type()), targetObject);
                
                if (pset->is(Type::IfcPropertySet)) {
                    format_entity_instance(pset, to, true);
                }
                if (pset->is(Type::IfcElementQuantity)) {
                    format_entity_instance(pset, to, true);
                }
            }

#ifdef USE_IFC4
            IfcTypeObject::list::ptr types = get_related<IfcObject, IfcRelDefinesByType, IfcTypeObject>(
                object, &IfcObject::IsTypedBy, &IfcRelDefinesByType::RelatingType
            );
#else
            IfcTypeObject::list::ptr types = get_related<IfcObject, IfcRelDefinesByType, IfcTypeObject>(
                object, &IfcObject::IsDefinedBy, &IfcRelDefinesByType::RelatingType
            );
#endif

            for (IfcTypeObject::list::it it = types->begin(); it != types->end(); ++it) {
                IfcTypeObject* type = *it;

                json::reference to = getEmptyObjectReferenceInArray(Type::ToString(type->type()), targetObject);
                format_entity_instance(type, to, true);
            }
        }

        // Handle IfcProduct mapping
        if (product->is(Type::IfcProduct)) {
            std::map<std::string, IfcPresentationLayerAssignment*> layers = IfcGeom::Kernel::get_layers(product->as<IfcProduct>());

            for (std::map<std::string, IfcPresentationLayerAssignment*>::const_iterator it = layers.begin(); it != layers.end(); ++it) {
                // IfcPresentationLayerAssignments don't have GUIDs (only optional Identifier) so use name as the ID.
                // Note that the IfcPresentationLayerAssignment passed here doesn't really matter as as_link is true for the format_entity_instance() call.
                json::reference to = getEmptyObjectReferenceInArray(Type::ToString(it->second->type()), targetObject);
                
                to["@{http://www.w3.org/1999/xlink}href"] = "#" + it->first;
                format_entity_instance(it->second, to, true);
            }

            IfcRelAssociates::list::ptr associations = product->HasAssociations();

            for (IfcRelAssociates::list::it it = associations->begin(); it != associations->end(); ++it) {
                if ((*it)->as<IfcRelAssociatesMaterial>()) {
                    IfcMaterialSelect* mat = (*it)->as<IfcRelAssociatesMaterial>()->RelatingMaterial();

                    json::reference to = getEmptyObjectReferenceInArray(Type::ToString(((IfcUtil::IfcBaseEntity*) mat)->type()), targetObject);

                    to["@{http://www.w3.org/1999/xlink}href"] = "#" + qualify_unrooted_instance(mat);
                    format_entity_instance((IfcUtil::IfcBaseEntity*) mat, to, true);
                }
            }
        }
    }

    // Format IfcProperty instances and insert into the DOM. IfcComplexProperties are flattened out.
    void format_properties(const IfcProperty::list::ptr& properties, json::reference jsonObject) {
        for (IfcProperty::list::it it = properties->begin(); it != properties->end(); ++it) {
            IfcProperty* p = *it;

            if (p->is(Type::IfcComplexProperty)) {
                IfcComplexProperty* complex = (IfcComplexProperty*) p;
                format_properties(complex->HasProperties(), jsonObject);
            }
            else {
                json::reference targetObject = getEmptyObjectReferenceInArray(Type::ToString(p->type()), jsonObject);
                format_entity_instance(p, targetObject);
            }
        }
    }

    // Format IfcElementQuantity instances and insert into the DOM.
    void format_quantities(const IfcPhysicalQuantity::list::ptr& quantities, json::reference jsonObject) {
        for (IfcPhysicalQuantity::list::it it = quantities->begin(); it != quantities->end(); ++it) {
            IfcPhysicalQuantity* p = *it;

            json::reference targetObject = getEmptyObjectReferenceInArray(Type::ToString(p->type()), jsonObject);
            
            format_entity_instance(p, targetObject);

            if (p->is(Type::IfcPhysicalComplexQuantity)) {
                IfcPhysicalComplexQuantity* complex = (IfcPhysicalComplexQuantity*)p;
                format_quantities(complex->HasQuantities(), targetObject);
            }
        }
    }

    // Writes string value to json reference, if string is empty it writes nullptr
    void add_string(json::reference ref, std::string stringToAdd) {
        if (stringToAdd.length() == 0)
            ref = nullptr;
        else
            ref = stringToAdd;
    }

    // Writes string vector to json reference, filters empty string values
    void add_string_vector(json::reference ref, const std::vector<std::string>& vectorToAdd) {
        ref = filter_empty_strings(vectorToAdd);
    }
    
    void log_error(const char *error) {
        std::stringstream ss;
        ss << "Failed to get ifc file header data, error: '" << error << "'";
        Logger::Message(Logger::LOG_ERROR, ss.str());
    }
} // ~unnamed namespace

void JsonSerializer::writeHeader(json::reference ifc) {
    json::reference header = ifc["header"];
    json::reference fileDescription = header["file_description"];
    json::reference fileName = header["file_name"];
    json::reference fileSchema = header["file_schema"];
    json::reference ifcJsonVersion = header["ifc_json_version"];

    add_string_vector(fileDescription["description"], file->header().file_description().description());
    add_string_vector(fileName["author"], file->header().file_name().author());
    add_string_vector(fileName["organization"], file->header().file_name().organization());
    add_string_vector(fileSchema["schema_identifiers"], file->header().file_schema().schema_identifiers());

    // TODO: Refactor to reduce repeated code
    try {
        add_string(fileDescription["implementation_level"], file->header().file_description().implementation_level());
    }
    catch (const IfcParse::IfcException& ex) {
        log_error(ex.what());
    }

    try {
        add_string(fileName["name"], file->header().file_name().name());
    }
    catch (const IfcParse::IfcException& ex) {
        log_error(ex.what());
    }

    try {
        add_string(fileName["time_stamp"], file->header().file_name().time_stamp());
    }
    catch (const IfcParse::IfcException& ex) {
        log_error(ex.what());
    }

    try {
        add_string(fileName["preprocessor_version"], file->header().file_name().preprocessor_version());
    }
    catch (const IfcParse::IfcException& ex) {
        log_error(ex.what());
    }

    try {
        add_string(fileName["originating_system"], file->header().file_name().originating_system());
    }
    catch (const IfcParse::IfcException& ex) {
        log_error(ex.what());
    }

    try {
        add_string(fileName["authorization"], file->header().file_name().authorization());
    }
    catch (const IfcParse::IfcException& ex) {
        log_error(ex.what());
    }

    add_string(ifcJsonVersion, IFC_JSON_VERSION);
}

void JsonSerializer::finalize() {
	argument_name_map.insert(std::make_pair("GlobalId", "id"));

	IfcProject::list::ptr projects = file->entitiesByType<IfcProject>();
	if (projects->size() != 1) {
		Logger::Message(Logger::LOG_ERROR, "Expected a single IfcProject");
		return;
	}
	IfcProject* project = *projects->begin();

    json jsonRoot;
    json::reference ifc = jsonRoot["ifc"];

	// Write the SPF header
    writeHeader(ifc);

	// Write the decomposition
	descend(project, ifc["decomposition"]);

    // Write all property sets and values
    IfcPropertySet::list::ptr psets = file->entitiesByType<IfcPropertySet>();
    json::reference properties = ifc["properties"];

    for (IfcPropertySet::list::it it = psets->begin(); it != psets->end(); ++it) {
        IfcPropertySet* pset = *it;

        json::reference propertyObject = getEmptyObjectReferenceInArray(Type::ToString(pset->type()), properties);
        
        format_entity_instance(pset, propertyObject);
        format_properties(pset->HasProperties(), propertyObject);
    }

    // Write all quantities and values
    IfcElementQuantity::list::ptr qtosets = file->entitiesByType<IfcElementQuantity>();
    json::reference quantities = ifc["quantities"];

    for (IfcElementQuantity::list::it it = qtosets->begin(); it != qtosets->end(); ++it) {
        IfcElementQuantity* qto = *it;

        json::reference quantityObject = getEmptyObjectReferenceInArray(Type::ToString(qto->type()), quantities);

        format_entity_instance(qto, quantityObject);
        format_quantities(qto->Quantities(), quantityObject);
    }

    // Write all type objects
    IfcTypeObject::list::ptr type_objects = file->entitiesByType<IfcTypeObject>();
    json::reference types = ifc["types"];

    for (IfcTypeObject::list::it it = type_objects->begin(); it != type_objects->end(); ++it) {
        IfcTypeObject* type_object = *it;

        descend(type_object, types);
    }

    // Write all assigned units
    IfcEntityList::ptr unit_assignments = project->UnitsInContext()->Units();
    json::reference units = ifc["units"];

    for (IfcEntityList::it it = unit_assignments->begin(); it != unit_assignments->end(); ++it) {
        json::reference unitObject = getEmptyObjectReferenceInArray(Type::ToString((*it)->type()), units);

        if ((*it)->is(IfcSchema::Type::IfcNamedUnit)) {
            IfcSchema::IfcNamedUnit* named_unit = (*it)->as<IfcSchema::IfcNamedUnit>();

            format_entity_instance(named_unit, unitObject);
            unitObject["@SI_equivalent"] = IfcParse::get_SI_equivalent(named_unit);
        }
        else if ((*it)->is(IfcSchema::Type::IfcMonetaryUnit)) {
            format_entity_instance((*it)->as<IfcSchema::IfcMonetaryUnit>(), unitObject);
        }
    }

    // Write presentation layers 
    // IfcPresentationLayerAssignments don't have GUIDs (only optional Identifier)
    // so use names as the IDs and only insert those with unique names. In case of possible duplicate names/IDs
    // the first IfcPresentationLayerAssignment occurrence takes precedence.
    std::set<std::string> layer_names;
    IfcPresentationLayerAssignment::list::ptr layer_assignments = file->entitiesByType<IfcPresentationLayerAssignment>();
    json::reference layers = ifc["layers"];

    for (IfcPresentationLayerAssignment::list::it it = layer_assignments->begin(); it != layer_assignments->end(); ++it) {
        const std::string& name = (*it)->Name();

        if (layer_names.find(name) == layer_names.end()) {
            layer_names.insert(name);

            json::reference layerObject = getEmptyObjectReferenceInArray(Type::ToString((*it)->type()), layers);
            layerObject["@id"] = name;
            format_entity_instance(*it, layerObject);
        }
    }
    
    // Write materials
    IfcRelAssociatesMaterial::list::ptr material_associations = file->entitiesByType<IfcRelAssociatesMaterial>();
    std::set<IfcMaterialSelect*> emitted_materials;
    json::reference materials = ifc["materials"];

    for (IfcRelAssociatesMaterial::list::it it = material_associations->begin(); it != material_associations->end(); ++it) {
        IfcMaterialSelect* mat = (**it).RelatingMaterial();
        if (emitted_materials.find(mat) == emitted_materials.end()) {
            emitted_materials.insert(mat);

            // Write IfcMaterialLayerSetUsage / IfcMaterialLayerSet
            if (mat->as<IfcMaterialLayerSetUsage>() || mat->as<IfcMaterialLayerSet>()) {
                IfcMaterialLayerSet* layerset = mat->as<IfcMaterialLayerSet>();
                std::string materialJsonKey = "IfcMaterialLayerSet";

                if (!layerset) {
                    materialJsonKey = "IfcMaterialLayerSetUsage";
                    layerset = mat->as<IfcMaterialLayerSetUsage>()->ForLayerSet();
                }

                json::reference materialObject = getEmptyObjectReferenceInArray(materialJsonKey, materials);
                materialObject["@id"] = qualify_unrooted_instance(mat);

                if (layerset->hasLayerSetName()) {
                    materialObject["@LayerSetName"] = layerset->LayerSetName();
                }

                IfcMaterialLayer::list::ptr ls = layerset->MaterialLayers();

                for (IfcMaterialLayer::list::it jt = ls->begin(); jt != ls->end(); ++jt) {
                    json::reference subMaterialObject = getEmptyObjectReferenceInArray(Type::ToString((*jt)->type()), materialObject);
                    
                    if ((*jt)->hasMaterial()) {
                        subMaterialObject["@Name"] = (*jt)->Material()->Name();
                    }
                    
                    format_entity_instance(*jt, subMaterialObject);
                }

                format_entity_instance((IfcUtil::IfcBaseEntity*) mat, materialObject);
            }
            // write IfcMaterialList
            else if (mat->as<IfcMaterialList>()) {
                IfcMaterial::list::ptr mats = mat->as<IfcMaterialList>()->Materials();

                json::reference materialObject = getEmptyObjectReferenceInArray("IfcMaterialList", materials);
                materialObject["@id"] = qualify_unrooted_instance(mat);

                for (IfcMaterial::list::it jt = mats->begin(); jt != mats->end(); ++jt) {
                    json::reference subMaterialObject = getEmptyObjectReferenceInArray(Type::ToString((*jt)->type()), materialObject);

                    format_entity_instance(*jt, subMaterialObject);
                }

                format_entity_instance((IfcUtil::IfcBaseEntity*) mat, materialObject);
            }
            // write IfcMaterial
            else if (mat->as<IfcMaterial>()) {
                json::reference materialObject = getEmptyObjectReferenceInArray("IfcMaterial", materials);
                materialObject["@id"] = qualify_unrooted_instance(mat);

                format_entity_instance((IfcUtil::IfcBaseEntity*) mat, materialObject);
            }
        }
    }

	std::ofstream f(IfcUtil::path::from_utf8(json_filename).c_str());

	// Write json to stream
    f << jsonRoot << std::endl;
}
