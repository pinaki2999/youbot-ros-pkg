/*
 * LogFileParsers.cpp
 *
 *  Created on: Dec 1, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "LogFileParsers.h"

using namespace std;
LogFileParsers::LogFileParsers() {
	// TODO Auto-generated constructor stub

}

LogFileParsers::~LogFileParsers() {
	// TODO Auto-generated destructor stub
}

void LogFileParsers::readLogFile(string filename){
	currentFilename = filename;
	ifstream inputFileStream;
	inputFileStream.open(filename.c_str(),ifstream::in);
	string s;

	if(inputFileStream.is_open()){
		while(getline(inputFileStream, s)){
			vector<string> tempvector;
			boost::split(tempvector, s, boost::is_any_of("\t "));
			data.push_back(tempvector);
		}
	} else {
		inputFileStream.close();
		cout<< filename <<" --- > File Not found" <<endl;
		exit(0);
	}
	inputFileStream.close();
}

void LogFileParsers::printData(){
	cout << "Total NUmber of Lines: " << data.size() << std::endl;
	for (unsigned int i=0; i<data.size();i++){
		cout << data[i].size()<< "\t";
	}
}


void LogFileParsers::writeData(string fileName){

	ofstream outputFileStream;
	outputFileStream.open(fileName.c_str());
	if(outputFileStream.is_open()){
		for (unsigned int i=0; i<data.size();i++){
				outputFileStream << data[i][0];
				for(size_t j=1; j<data[i].size(); j++){
					outputFileStream << "\t" << data[i][j];
					}
				outputFileStream << std::endl;

			}
	}

	outputFileStream.close();
}

void removeOffsetErrors(vector< vector<string> > &data, string columns){

	vector<string> vecColumns;
	boost::split(vecColumns, columns, boost::is_any_of(" "));

	double offsets[vecColumns.size()];

	//initialize the offsets to 0
	for(size_t j=0; j<vecColumns.size(); j++){
				offsets[j]=0;
	}

	//calculate the offsets
	for (size_t i =0; i< data.size(); i++){
		for(size_t j=0; j<vecColumns.size(); j++){
			offsets[j] += atof(data[i][atoi(vecColumns[j].c_str())].c_str());
		}
	}
	cout << "Found Offsets are: " << endl;
	for(size_t j=0; j<vecColumns.size(); j++){
				offsets[j] /= data.size();
				cout << vecColumns[j] << " : " << offsets[j] << endl;
	}

	//Remove offsets from the data
	stringstream tempValues;
	for (size_t i =0; i< data.size(); i++){
		for(size_t j=0; j<vecColumns.size(); j++){
			tempValues.str("");
			tempValues << atof(data[i][atoi(vecColumns[j].c_str())].c_str())-offsets[j];
			data[i][atoi(vecColumns[j].c_str())] = tempValues.str();
		}
	}


}

int main(int argc, char* argv[]){

	LogFileParsers logFileParser;
	logFileParser.readLogFile(argv[argc-1]);
	logFileParser.printData();

	removeOffsetErrors(logFileParser.data,"6 7 8 9");

	stringstream outputFileName;
	outputFileName.str("");
	outputFileName << argv[argc-1] << "_offset_removed";
	logFileParser.writeData(outputFileName.str());
}
