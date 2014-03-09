#include <iostream>
#include <stdio.h>
#include <vector>

#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <list>
#include <sys/timeb.h>

using namespace std;

//Delimiters
char DELIMITER_GROUP = 29;
char DELIMITER_RECORD = 30;


std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

double string_to_double( const std::string& s )
{
    istringstream i(s);
    double x;
    if (!(i >> x))
        return 0;
    return x;
}

int sgn(double x)
{
    return (x > 0) - (x < 0);
}

void parse (string line, vector<double> &position, vector<double> &angle)
{
    
    vector<string> groups, group1, group2;
    groups = split(line, DELIMITER_GROUP);

    if(groups.size() == 1)
    {
        //We only got the 3D position
        group1 = split(groups.at(0), DELIMITER_RECORD);

        for(int i = 0; i < group1.size(); ++i)
        {
            position.push_back(string_to_double(group1.at(i)));
        }
    }
    else if(groups.size() == 2)
    {
        //We have both 3D position and angle data
        group1 = split(groups.at(0), DELIMITER_RECORD);
        group2 = split(groups.at(1), DELIMITER_RECORD);

        for(int i = 0; i < group1.size(); ++i)
        {
            position.push_back(string_to_double(group1.at(i)));
        }

        for(int i = 0; i < group2.size(); ++i)
        {
            angle.push_back(string_to_double(group2.at(i)));
        }
    }
    else
    {
        //We have no idea!
    }
}

 double regulateDistance(double distance, double prevDistance, double previousOutput, double reference, double Kp, double Ki, double deltaT)   
{
    double error = distance-reference;
    double prevError = prevDistance - reference;
    double output = Kp*error + Ki*deltaT*(error + prevError);
    if (abs(output)>100)
    {
        return sgn(output)*100;
    }
    else
    {
        return output;
    }
}

// FPS - Get count in millisoconds
int getMilliCount(){
    timeb tb;
    ftime(&tb);
    int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
    return nCount;
}

// FPS - Calculate the difference between start and end time.
int getMilliSpan(int timeStart){
    int span = getMilliCount() - timeStart;
    if (span < 0)
        span += 0x100000 * 1000;
    return span;
}


int main(int argc, char** argv)
{
    bool loop = true, readLine = true;
    string line;
    double startTime;

    double previousOutputX = 0, previousOutputZ = 0;
    vector<double> position (3), angle (3);
    list<vector<double>> positions, angles;

    while(loop)
    {
        try
        {
            position.clear();
            angle.clear();

            //calculate delta time
            double deltaT = (double)getMilliSpan(startTime) / 1000;
            startTime = getMilliCount();

            //Read input
            getline(cin, line);

            parse(line, position, angle);

            if(positions.size() >= 2)
            {
                positions.pop_back();
            }
            positions.push_front(position);
            if(angles.size() >= 2)
            {
                angles.pop_back();
            }
            angles.push_front(position);
            
            
            //regulate z position
            if(positions.size() == 2 && angles.size() == 2)
            {
                double outputX = regulateDistance(positions.front().at(0), positions.back().at(0), previousOutputX, 0, 0.2, 0.2, deltaT);
                double outputZ = regulateDistance(positions.front().at(2), positions.back().at(2), previousOutputZ, 100, 0.2, 0.2, deltaT);
                cout << "move in X: " << outputX << "%\n";
                cout << "move in Z: " << outputZ << "%\n";
                previousOutputX = outputX;
                previousOutputZ = outputZ;
            }

            
        }
        catch (...)
        {
            cout << "Exception!" << endl;
            readLine = false;
        }
        
    }
    return 0;
}