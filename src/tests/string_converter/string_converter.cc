
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

#include "string_converter.h"

#include <sstream>
#include <iostream>

StringConverter::StringConverter() {
}

StringConverter::~StringConverter() {
}

bool StringConverter::convert(std::string value, bool default_value) {

	if(value.compare("true") == 0) {
		return true;
	} else if(value.compare("false") == 0) {
		return false;
	} else {
		return default_value;
	}
}


long StringConverter::convert(std::string value, long default_value) {

	std::stringstream stream(value);

	long result;
	stream >> result;

	if(stream.fail()) {
		return default_value;
	} else {
		return result;
	}
}

int StringConverter::convert(std::string value, int default_value) {

	std::stringstream stream(value);

	int result;
	stream >> result;

	if(stream.fail()) {
		return default_value;
	} else {
		return result;
	}
}

unsigned int StringConverter::convert(std::string value, unsigned int default_value) {

	std::stringstream stream(value);

	unsigned int result;
	stream >> result;

	if(stream.fail()) {
		return default_value;
	} else {
		return result;
	}
}

float StringConverter::convert(std::string value, float default_value) {

	std::stringstream stream(value);

	float result;
	stream >> result;

	if(stream.fail()) {
		return default_value;
	} else {
		return result;
	}
}

double StringConverter::convert(std::string value, double default_value) {

	std::stringstream stream(value);

	double result;
	stream >> result;

	if(stream.fail()) {
		return default_value;
	} else {
		return result;
	}
}
