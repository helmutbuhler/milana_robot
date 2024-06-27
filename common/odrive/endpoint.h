// This class holds ODrive endpoints (or any endpoint really) in a std::map  
// which contains the endpoint name and endpoint id. Each endpoint can also 
// have a child (root has child, 'motor0'. 'motor0' has child, 'pos_setpoint').
// This class also implements the abstraction from the ODrive class, essentially
// providing a direct interface from an Endpoint to a endpoint on the ODrive hardware.

#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <iterator>
#include <assert.h>
#include "../../common/helper.h" // This is just needed for basic int type definitions, you can copy those into this file if you want to use this in your project

class ODrive;

// Helper int to store a fw version in a single integer
using ODriveVersion = int;
const ODriveVersion odrive_version_0_5_1 = 5001;
const ODriveVersion odrive_version_0_5_6 = 5006;

class Endpoint
{
public:
	ODrive* odrive = nullptr;
	int id = -1;
	std::string name;
	std::string type;
	std::string access;
	std::map<std::string, Endpoint> children;

public:
	Endpoint& operator() (const char* name);

	bool is_valid() const;
	bool has_children() const;
	bool has_child(const char* name) const;

	void set(float value) const;
	void set(s32 value) const;
	void set(s64 value) const;
	void set(bool value) const;
	void get(float& value) const;
	void get(u8& value) const;
	void get(s32& value) const;
	void get(s64& value) const;
	void get(u64& value) const;
	void get(bool& value) const;

	// Alternative getter that returns the value directly
	template<typename T>
	T get2() const
	{
		T value = T();
		get(value);
		return value;
	}


	// Call a function on ODrive.
	// You first supply all the parameters, and if the function has a return value, you pass
	// a pointer to the return value as the last parameter.
	// For example, this is how you call a function 'float foo(int)':
	// float ret;
	// odrive.root("foo").call(123, &ret);
	// Every possible parameter/return combination that can be used is defined in ODrive::call;
	// If you need one that's not there yet, just add it there.
	// 
	// About the hack typename in the template parameter list:
	// It's there to make the odrive expression in the body dependent on the template.
	// Otherwise the C++ compiler might complain because ODrive isn't defined here yet.
	// (And the MS compiler will complain otherwise)
	// Don't set hack to anything when calling this!
	template<class... Args, typename hack = Endpoint>
	void call(Args&&... args) const
	{    
		if (!has_children() && is_valid() && type == "function")
		{
			((hack*)this)->odrive->call(id, std::forward<Args>(args)...);
			return;
		}
		((hack*)this)->odrive->communication_error = true;
		printf("expected %s to be a function\n", name.c_str());
	}

	ODriveVersion get_odrive_fw_version();
	bool odrive_fw_is_milana(); // Is ODrive flashed with the Milana fw branch?
};
