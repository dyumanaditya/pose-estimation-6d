/**
 * @file csv_parser.h
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Header file for CSV parser
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */

#ifndef CSV_PARSER_H
#define CSV_PARSER_H

#include <string>
#include <fstream>
#include <vector>
#include <Eigen/Dense>


/**
 * @brief Reads/Writes from the csv file that contains the poses of the object for each frame
 * 
 */
class CSVParser
{
private:
    /**
     * @brief Out stream object to write the poses to the csv file
     */
    std::ofstream csv;
    
public:
    /**
     * @brief Load transformation paramters from a csv file into an Eigen Matrix
     * 
     * @tparam M 
     * @param path Path to the csv file
     * @return M 
     */
	template<typename M> 
	inline M static readPoseFromCSV (const std::string & path)
	{
    	std::ifstream indata;
	    indata.open(path);
	    std::string line;
	    std::vector<double> values;
	    uint rows = 0;
	    while (std::getline(indata, line)) {
	        std::stringstream lineStream(line);
	        std::string cell;
	        while (std::getline(lineStream, cell, ',')) {
	            values.push_back(std::stod(cell));
	        }
	        ++rows;
    	}
    	return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
	}

    /**
     * @brief Construct a new CSVParser object
     * 
     * @param csv_path Path of csv file in which poses are to be written
     * @param csv_name Name of csv file in which poses are to be written
     */
    CSVParser(std::string csv_path, std::string csv_name);

    /**
     * @brief Destroy the CSVParser object
     * 
     */
    ~CSVParser();

    /**
     * @brief Writes the pose contained inside the 4x4 transformation matrix to
     * the csv file in the form: x,y,z,q1,q2,q3,q4
     * 
     * @param transformation The 4x4 transformation matrix that represents the aligned object's transformation
     */
    void writePoseToCsv(Eigen::Matrix4f transformation);
};


#endif