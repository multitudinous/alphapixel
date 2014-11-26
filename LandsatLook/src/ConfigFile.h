// MTL metadata file parse tool derived from 
// http://www.dreamincode.net/forums/topic/183191-create-a-simple-configuration-file-parser/
// Modified by Gary Huber for AlphaPixel and AgSolver to allow re-use of a key 
// and not exiting if the = separator is not found on a line
// which is normal for the last line of an MTL file.

#include <iostream>
#include <string>
#include <sstream>
#include <map>
#include <fstream>

namespace landsatlook {

void exitWithError(const std::string &error, bool exitNow = true );

class Convert
{
public:
	template <typename T>
	static std::string T_to_string(T const &val) 
	{
		std::ostringstream ostr;
		ostr << val;

		return ostr.str();
	}
		
	template <typename T>
	static T string_to_T(std::string const &val) 
	{
		std::istringstream istr(val);
		T returnVal;
		if (!(istr >> returnVal))
			exitWithError("CFG: Not a valid " + (std::string)typeid(T).name() + " received!\n");

		return returnVal;
	}

	template <>
	static std::string string_to_T(std::string const &val)
	{
		return val;
	}
};

class ConfigFile
{
private:
	std::map<std::string, std::string> contents;
	std::string fName;

	void removeComment(std::string &line) const;

	bool onlyWhitespace(const std::string &line) const;

	bool validLine(const std::string &line) const;

	void extractKey(std::string &key, size_t const &sepPos, const std::string &line) const;

	void extractValue(std::string &value, size_t const &sepPos, const std::string &line) const;

	void extractContents(const std::string &line);

	void parseLine(const std::string &line, size_t const lineNo);

	void ExtractKeys();

public:
	ConfigFile(const std::string &fName);

	bool keyExists(const std::string &key) const;

	template <typename ValueType>
	ValueType getValueOfKey(const std::string &key, ValueType const &defaultValue = ValueType()) const
	{
		if (!keyExists(key))
			return defaultValue;

		return Convert::string_to_T<ValueType>(contents.find(key)->second);
	}

};

}