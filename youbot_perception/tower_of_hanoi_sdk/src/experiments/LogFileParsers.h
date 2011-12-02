/*
 * LogFileParsers.h
 *
 *  Created on: Dec 1, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef LOGFILEPARSERS_H_
#define LOGFILEPARSERS_H_
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>
using namespace std;

class LogFileParsers {
private:

	string currentFilename;


public:
	LogFileParsers();
	virtual ~LogFileParsers();

	void readLogFile(string fileName);
	void printData();
	void writeData(string fileName);


	vector< vector<string> > data;
};

#endif /* LOGFILEPARSERS_H_ */
