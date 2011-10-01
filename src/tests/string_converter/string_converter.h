
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class interprets the contents of a string and converts it into
 * different types.
 */

#ifndef STRING_CONVERTER_H_
#define STRING_CONVERTER_H_

#include <string>

class StringConverter {

public:
	StringConverter();
	virtual ~StringConverter();

	bool convert(std::string value, bool default_value);
	long convert(std::string value, long default_value);
	int convert(std::string value, int default_value);
	unsigned int convert(std::string value, unsigned int default_value);
	float convert(std::string value, float default_value);
	double convert(std::string value, double default_value);
};

#endif /*STRING_CONVERTER_H_*/
