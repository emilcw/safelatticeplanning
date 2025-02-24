#pragma once
#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <map>
#include <vector>
#include <string>
#include <ctime>
#include <algorithm> 
#include <cctype>
#include <locale>

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

std::string calculate_datetime() {
  std::time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  std::time (&rawtime);
  timeinfo = localtime(&rawtime);

  std::strftime(buffer,sizeof(buffer),"%d-%m-%Y_%I:%M:%S",timeinfo);
  
  return std::string(buffer);
}

class Logger {
public:
  Logger() : name(""), precision(std::numeric_limits<long double>::digits10) {}
  Logger(std::string name, int precision = std::numeric_limits<long double>::digits10) : name(name), precision(precision) {}

  void log(std::string tag, double time, double value) {
    logs[tag].push_back(std::pair<double,double>(time,value));
  }

  std::vector<std::pair<double,double>> get_log(std::string tag) {
    return logs[tag];
  }

  void write_to_file() {
    if(name == "") return;
    std::string full_path = path + name + ".m";
    std::ofstream file;
    try {
      file.open(full_path);
      file << "function [logs] = " << name << "()\n";
      file << "logs = {};\n";
      int k = 1;
      std::map<std::string,std::vector<std::pair<double,double> > >::iterator tag;
      for(tag = logs.begin(); tag != logs.end(); tag++, k++) {
        file << "logs{" << k << "} = struct('name','" << tag->first << "','value',[";
        file << std::setprecision(precision) << tag->second[0].first << "," 
		     << std::setprecision(precision) << tag->second[0].second;
        for(int n = 1; n < tag->second.size(); n++) {
          file << ";" << std::setprecision(precision) << tag->second[n].first
			   << "," << std::setprecision(precision) << tag->second[n].second;
        }
        file << "]);\n";
      }
      file << "end";
      file.close();
      std::cerr << "Saved log to path \"" << full_path << "\"." << std::endl;
    }
    catch (std::ios_base::failure& e) {
      std::cerr << "Failed to save log to path \"" << full_path << "\": " << e.what();
    }
  }

public:
  int precision;
  std::string name;
  std::string path;
  std::map<std::string,std::vector<std::pair<double,double> > > logs;
};



/* Test result 
-------------------------------------*/
#include "Profiler.h"
#include <sstream>
#include <fstream>

class TestResult {
public:
  Scenario scenario;
  double planning_time;
  double replanning_time;
  Profiler profiler;
  double search_time1;
  double search_time2;
  double total_time;
  int plan_size;
  int trajectory_size;
  double cost;
  int primitives_size;
  int open_set_size;
  int closed_set_size;
  std::string datetime;
  std::string comment;
};

std::string print_test_results(std::vector<TestResult> results, int mark_index = -1) {
  std::string datetime = calculate_datetime();
  std::stringstream ss;
  ss << std::left  << std::setw(4) << "#"
      << std::left  << std::setw(20) << "Scenario"
      << std::right << std::setw(9) << "Search1"
      << std::right << std::setw(9) << "Search2"
      << std::right << std::setw(9) << "Total"
      << std::right << std::setw(9) << "Cost"
      << std::right << std::setw(10) << "OpenSet"
      << std::right << std::setw(10) << "ClosedSet"
      << std::right << std::setw(7) << "Plan"
      << std::right << std::setw(8) << "Traj"
      << std::right << std::setw(14) << "#Primitives" 
      << std::right << std::setw(2) << "  " 
      << std::left << std::setw(18) << "DateTime"
      << std::left << std::setw(2) << "  " 
      << std::left << std::setw(10) << "Comments" << std::endl;
  for(int n = 0; n < results.size(); n++) {
    TestResult & result = results[n];
    if (result.datetime == "") {
      result.datetime = datetime;
    }
    std::stringstream numbering;
    numbering << "" << n+1 << "";
    if(mark_index >= 0 && n >= mark_index) {
      numbering << "*";
    }
    ss  << std::left << std::setw(4) << numbering.str()
        << std::left << std::setw(20) << result.scenario.name
        << std::setprecision(6)
        << std::right << std::setw(9) << result.search_time1
        << std::right << std::setw(9) << result.search_time2
        << std::right << std::setw(8) << result.total_time
        << std::right << std::setw(9) << result.cost
        << std::right << std::setw(10) << result.open_set_size
        << std::right << std::setw(10) << result.closed_set_size
        << std::right << std::setw(7) << result.plan_size
        << std::right << std::setw(8) << result.trajectory_size
        << std::right << std::setw(14) << result.primitives_size
        << std::right << std::setw(3) << "  " 
        << std::right << std::setw(18) << result.datetime
        << std::left << std::setw(2) << "  ";
        if(result.comment.size() > 0) {
          ss << std::left << std::setw(10) << result.comment;
        }
        ss << std::endl;
  }
  return ss.str();
}

std::string print_test_result(TestResult test_result) {
  std::vector<TestResult> results = {test_result};
  return print_test_results(results);
}

std::vector<TestResult> read_test_results(std::string path) {
  std::vector<TestResult> results;
  std::string line;
  std::ifstream file(path.c_str());
  if(!file.is_open()) {
    ROS_ERROR_STREAM("Could not open Test Result file on path \"" << path << "\" for reading.");
    return results;
  }
  std::getline(file,line);  // Read header
  while(std::getline(file,line)) {
    stringstream ss(line);
    TestResult result;
    int dummy;
    ss >> dummy
       >> result.scenario.name
       >> result.search_time1
       >> result.search_time2
       >> result.total_time
       >> result.cost
       >> result.open_set_size
       >> result.closed_set_size
       >> result.plan_size
       >> result.trajectory_size
       >> result.primitives_size
       >> result.datetime;
       std::getline(ss,result.comment);
       trim(result.comment);
       
    results.push_back(result);
  }
  file.close();
  return results;
}

void write_test_results(std::string path, std::vector<TestResult> results, int mark_index = -1) {
  std::ofstream file(path.c_str());
  if(!file.is_open()) {
    ROS_ERROR_STREAM("Could not open Test Result file on path \"" << path << "\" for writing.");
    return;
  }
  file << print_test_results(results);
  file.close();
}




#endif // __LOGGER_H__
