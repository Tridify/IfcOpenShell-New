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

#ifndef JSONSERIALIZER_H
#define JSONSERIALIZER_H

#include "../ifcconvert/Serializer.h"
#include "json.hpp"

class JsonSerializer : public Serializer {
private:
	IfcParse::IfcFile* file;
	std::string json_filename;
    static const std::string IFC_JSON_VERSION;
public:
    JsonSerializer(const std::string& json_filename)
		: Serializer()
		, json_filename(json_filename)
	{}

	bool ready() { return true; }
	void writeHeader() {}
	void writeHeader(nlohmann::json::reference& ref);
	void finalize();
	void setFile(IfcParse::IfcFile* f) { file = f; }
};

#endif