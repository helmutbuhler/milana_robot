#include "endpoint.h"
#include "ODrive.h"


Endpoint& Endpoint::operator() (const char* name) {
	auto it = children.find(name);
	if (it == children.end())
	{
		printf("odrive: cannot find %s in %s!\n", name, this->name.size() ? this->name.c_str() : "root");
		odrive->communication_error = true;
		return *this;
	}
	return it->second;
}

bool Endpoint::is_valid() const {
	return id != -1;
}

bool Endpoint::has_children() const {
	return !children.empty();
}

bool Endpoint::has_child(const char* name) const {
	return children.find(name) != children.end();
}

void Endpoint::set(float value) const {
	if (!has_children() && is_valid() && type == "float")
	{
		odrive->set_value(id, value);
	}
	else
	{
		printf("Cannot write float %s. ID: %i access: %s type: %s\n", name.c_str(), id, access.c_str(), type.c_str());
		odrive->communication_error = true;
	}
}


void Endpoint::set(s32 value) const {
	if (!has_children() && is_valid())
	{
		if (type == "uint32" || type == "int32")
		{
			odrive->set_value(id, value);
			//printf("Set ID %i with float: %d\n", id, value);
			return;
		}
		else if (type == "uint8")
		{
			assert(value >= 0 && value < 256);
			u8 val = value;
			odrive->set_value(id, val);
			return;
		}
		else if (type == "int8")
		{
			assert(value >= -128 && value < 128);
			s8 val = value;
			odrive->set_value(id, val);
			return;
		}
		else if (type == "uint16")
		{
			assert(value >= 0 && value < 65536);
			u16 val = value;
			odrive->set_value(id, val);
			return;
		}
		else if (type == "int16")
		{
			assert(value >= -32768 && value < 32768);
			s16 val = value;
			odrive->set_value(id, val);
			return;
		}
		else if (type == "uint64" || type == "uint32")
		{
			s64 val = value;
			odrive->set_value(id, val);
			return;
		}
	}
	printf("Cannot write int %s. ID: %i access: %s type: %s\n", name.c_str(), id, access.c_str(), type.c_str());
	odrive->communication_error = true;
}

void Endpoint::set(s64 value) const {
	if (!has_children() && is_valid())
	{
		if (type == "uint64" || type == "int64")
		{
			odrive->set_value(id, value);
			return;
		}
	}
	printf("Cannot write s64 %s. ID: %i access: %s type: %s\n", name.c_str(), id, access.c_str(), type.c_str());
	odrive->communication_error = true;
}

void Endpoint::set(bool value) const {
	if (!has_children() && is_valid() && type == "bool")
	{
		odrive->set_value(id, value);
		//printf("Set ID %i with float: %d\n", id, value);
	}
	else
	{
		odrive->communication_error = true;
		printf("Cannot write bool %s. ID: %i access: %s type: %s\n", name.c_str(), id, access.c_str(), type.c_str());
	}
}

void Endpoint::get(float& value) const {
	if (!has_children() && is_valid()) {
		if (type == "float") {
			odrive->get_value(id, value);
			//printf("got %.2f from ID %i\n", value, id);
			return;
		}
	}
	printf("Cannot read float %s. ID: %i access: %s type: %s\n", name.c_str(), id, access.c_str(), type.c_str());
	odrive->communication_error = true;
}

void Endpoint::get(s32& value) const {
	if (!has_children() && is_valid()) {
		if (type == "uint32" || type == "int32") {
			odrive->get_value(id, value);
			//printf("got int32 %d from ID %i %s\n", (int)value, id, name.c_str());
			return;
		}
		else if (type == "uint16" || type == "int16") {
			s16 val = 0;
			odrive->get_value(id, val);
			//printf("got %d from ID %i\n", value, id);
			value = val;
			return;
		}
		else if (type == "uint8" || type == "int8") {
			s8 val = 0;
			odrive->get_value(id, val);
			//printf("got u8 %d from ID %i %s\n", (int)value, id, name.c_str());
			value = val;
			return;
		}
	}
	printf("Cannot read int %s. ID: %i access: %s type: %s\n", name.c_str(), id, access.c_str(), type.c_str());
	odrive->communication_error = true;
}

void Endpoint::get(u8& value) const {
	if (!has_children() && is_valid()) {
		if (type == "uint8" || type == "int8") {
			odrive->get_value(id, value);
			//printf("got %d from ID %i\n", value, id);
			return;
		}
	}
	printf("Cannot read u8 %s. ID: %i access: %s type: %s\n", name.c_str(), id, access.c_str(), type.c_str());
	odrive->communication_error = true;
}

void Endpoint::get(s64& value) const {
	if (!has_children() && is_valid()) {
		if (type == "uint64" || type == "int64") {
			odrive->get_value(id, value);
			//printf("got %d from ID %i\n", value, id);
			return;
		}
		else if (type == "uint32") {
			u32 val = 0;
			odrive->get_value(id, val);
			//printf("got %d from ID %i\n", value, id);
			value = val;
			return;
		}
		else if (type == "int32") {
			s32 val = 0;
			odrive->get_value(id, val);
			//printf("got %d from ID %i\n", value, id);
			value = val;
			return;
		}
		else if (type == "uint16" || type == "int16") {
			s16 val = 0;
			odrive->get_value(id, val);
			//printf("got %d from ID %i\n", value, id);
			value = val;
			return;
		}
		else if (type == "uint8" || type == "int8") {
			s8 val = 0;
			odrive->get_value(id, val);
			//printf("got u8 %d from ID %i %s\n", (int)value, id, name.c_str());
			value = val;
			return;
		}
	}
	printf("Cannot read s64 %s. ID: %i access: %s type: %s\n", name.c_str(), id, access.c_str(), type.c_str());
	odrive->communication_error = true;
}

void Endpoint::get(u64& value) const {
	if (!has_children() && is_valid()) {
		if (type == "uint64" || type == "int64") {
			odrive->get_value(id, value);
			//printf("got %d from ID %i\n", value, id);
			return;
		}
	}
	printf("Cannot read u64 %s. ID: %i access: %s type: %s\n", name.c_str(), id, access.c_str(), type.c_str());
	odrive->communication_error = true;
}

void Endpoint::get(bool& value) const {
	if (!has_children() && is_valid()) {
		if (type == "bool") {
			odrive->get_value(id, value);
			//printf("got %d from ID %i\n", value, id);
			return;
		}
	}
	printf("Cannot read bool %s. ID: %i access: %s type: %s\n", name.c_str(), id, access.c_str(), type.c_str());
	odrive->communication_error = true;
}

ODriveVersion Endpoint::get_odrive_fw_version()
{
	return odrive->odrive_fw_version;
}
bool Endpoint::odrive_fw_is_milana()
{
	return odrive->odrive_fw_is_milana;
}
