#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <queue>
#include <iomanip>
#include <regex>
#include <chrono>
#include <thread>
#include <stdlib.h>
#pragma once// STD Library
// Bridges Repository
#include "Bridges.h" // This class contains methods to connect and transmit a user's data structure representation to the Bridges server.
#include "DataSource.h" // This class provides an API to various data sources used in BRIDGES.
#include "GraphAdjList.h" // This class provides methods to represent adjacency list based graphs.
#include <data_src/OSMData.h> // Class that holds Open Street Map data, from https://openstreetmap.org

//Read through documentation again.!
using namespace std;
using namespace bridges;

//enums used to pass into functions, so that the function can work in different ways. like a pseudo-overload

enum inputType
{
    rect, // coordinate rectangle, four vars {x min, y min, x max, y max}
    point, // coordinate point, two vars {x, y}
    name // name of location, string
};

// Convert x and y values (0 - 100) that will translate to latitude and longitude to be used
vector<double> convertToLatLong(const OSMData& osm_data, double x, double y) {
    double latr[2];
    double lonr[2];
    osm_data.getLatLongRange(latr, lonr);

    vector<double> latlon(2);
    latlon[0] = y / 100.0 * (latr[1] - latr[0]) + latr[0];
    latlon[1] = x / 100.0 * (lonr[1] - lonr[0]) + lonr[0];
    return latlon;
}

// Find the vertex closest to the center of the map ---- O(n) time
int getCenter(const OSMData& osm_data) {
    double latr[2];
    double lonr[2];
    osm_data.getLatLongRange(latr, lonr);

    double lat = (latr[0] + latr[1]) / 2.;
    double lon = (lonr[0] + lonr[1]) / 2.;

    // Get all the vertices from the data
    vector<OSMVertex> vertices = osm_data.getVertices();

    double latV = 0;
    double lonV = 0;
    int index = -1;
    double min = INT_MAX;

    // Iterate through each vertex and compare distances of each to the center. 
    // The vertex with the smallest difference between coordinates gets saved as the closest one
    for (int i = 0; i < vertices.size(); i++)
    {
        latV = vertices[i].getLatitude();
        lonV = vertices[i].getLongitude();
        double dif = abs(lat - latV) + abs(lon - lonV);

        if (min > dif)
        {
            min = dif;
            index = i;
        }
    }
    return index;   // Gives the index of the vertex (within vertices vector or adjacencyList)
}

// ======== DIJKSTRA'S SHORTEST PATH IMPLEMENTATION ========= O( (V + E)log(V) ) time
vector<int> shortestPath(const GraphAdjList<int, OSMVertex, double>& gr, int source, unordered_map<int, int>& parent)
{
    // -- INITIALIZATION -- 
    int N = gr.getVertices()->size();   // Get the amount of vertices in the adjacency list

    // 1) INDEX: Keeps track of the vertex we are evaluating in dijkstras algorithm
    int index = source;

    // 2) PARENT: Unordered map that is passed into function call. Source vertex does NOT have a parent, initialize as -1
    parent[source] = -1;

    // 3) VISITED: A set that keeps track of which vertices we have already visited to not re-visit nodes that already have shortest distance
    unordered_set<int> visited;
    visited.insert(source); // Initialize source vertex as visited

    // 4) DISTANCES: the vector, d, that contains DISTANCES for each vertex. All initialized at infinity. Nodes that cannot be reached remain at INT_MAX after the algorithm completes
    vector<int> d(N, INT_MAX);
    d[index] = 0;

    // 5) MIN HEAP: Keeps track of the minimum DISTANCE and corresponding INDEX. Used to find the next index to visit during dijkstra's algorithm.
    //              A heap of pairs, where the pair stores <Distance, Index> as <double, int>
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    pq.push(make_pair(0, index));   // Initialize the source vertex

    // DIJKSTRA'S ALGORITHM
    while (!pq.empty())
    {
        pq.pop();

        // -- FIND NEIGHBORS --
        const SLelement<Edge<int, double>>* neighborPtr = gr.getAdjacencyList(index);

        while (neighborPtr != nullptr)  //  Iterate through each neighbor
        {

            Edge<int, double> edge = neighborPtr->getValue();   // get the edge object from linked list
            int n = edge.to();                                  // get INDEX of the neighbor
            double weight = edge.getEdgeData();                 // get the weight of the distance to this neighbor

            // RELAXATION
            if (d[n] > d[index] + weight)
            {
                d[n] = d[index] + weight;
                pq.push(make_pair(d[n], n));
                parent[n] = index;
            }

            neighborPtr = neighborPtr->getNext();
        }

        // The heap cannot "reassign" distance values. Thus there will be "outdated" distance values. Check to see if it was already visited and get rid of it if so.
        bool alreadyVisited = visited.find(pq.top().second) != visited.end();
        while (alreadyVisited && !pq.empty())
        {
            pq.pop();
            alreadyVisited = visited.find(pq.top().second) != visited.end();
        }

        index = pq.top().second;    // REASSIGN index value to NEW lowest distance (that wasn't already visisted)
        visited.insert(index);      // Insert this index into the set of already visited vertex indices
    }

    return d;   // Return the vector of DISTANCES
}

// return the vertex the closest to a particular (lat,lon). CHECK lat and long range before calling function ---- O(n) time
int getClosestVertex(vector<OSMVertex>& vertices, double lat, double lon)
{
    double latV = 0;
    double lonV = 0;
    int index = -1;
    double min = INT_MAX;

    // Iterate through each vertex and compare distances of each to the center. 
    // The vertex with the smallest difference between coordinates gets saved as the closest one
    for (int i = 0; i < vertices.size(); i++)
    {
        latV = vertices[i].getLatitude();
        lonV = vertices[i].getLongitude();
        double dif = abs(lat - latV) + abs(lon - lonV);

        if (min > dif)
        {
            min = dif;
            index = i;
        }

    }
    return index;
}
//styles the root vertex red, for easy identification
void styleRoot(GraphAdjList<int, OSMVertex, double>& graph, int root) {
    graph.getVertex(root)->setColor("red");
}
//styles the dest vertex purple, for easy identification
void styleDest(GraphAdjList<int, OSMVertex, double>& graph, int root) {
    graph.getVertex(root)->setColor("purple");
}
//change the style of of the root and dest, combines the prev two for ease of use
void styleEnds(GraphAdjList<int, OSMVertex, double>& graph, int root, int dest)
{
    styleRoot(graph, root);
    styleDest(graph, dest);
}

//style graph based on whether vertices and edges sit on the shortest path between dest and source.
void styleParent(GraphAdjList<int, OSMVertex, double>& graph,
    const std::unordered_map<int, int>& parent,
    int dest) 
{
    int prev = parent.at(dest);
    int child = dest;
    while (prev != -1)
    {
        graph.getVertex(child)->setColor("magenta");
        graph.getEdge(prev, child).setColor("pink");
        child = prev;
        prev = parent.at(child);
    }
    styleEnds(graph, child, dest);
}
// converts a stringstream& src into coords, knowing the # of coords from i
vector<double> setCoords(stringstream& src, inputType i)
{
    string x;
    vector<double> result;
    if (i == rect)
        result.resize(4);
    else
        result.resize(2);
    for (int i = 0; i < result.size(); i++)
    {
        getline(src, x, ','); //copies one coordinate into string x
        result[i] = stod(x); //converts one coordinate into a double
    }
    if (i == rect) //if min values are greater than max values, swaps them
    {
        if (result.at(0) > result.at(2))
            swap(result[0], result[2]);
        if (result[1] > result[3])
            swap(result[1], result[3]);
    }
    return result;
}
// main framework that the preset cities function on for user input. Allows both digit values and names to be input and then returns a value to be used in main.
int choiceParsing(stringstream& input)
{
    string buff;
    buff = input.str();
    regex optnDig("[0-9]\.?"); //matches a digit from 1-9 and then a possible . after
    regex optnCity("[a-zA-Z]+,? *([a-zA-Z]+)?");
    regex miami("miami,? *(florida)?", regex_constants::icase);
    regex dallas("dallas,? *(texas)?", regex_constants::icase);
    regex la("los angeles,? *(california)?", regex_constants::icase);
    regex seattle("seattle,? *(washington)?", regex_constants::icase);
    regex orleans("new\sorleans,? *(louisiana)?", regex_constants::icase);
    regex gainesville("gainesville,? *(florida)?", regex_constants::icase);
    regex newyork("new york(city)?,? *(new york)?", regex_constants::icase);
    
    vector<regex> options
    { miami, newyork, dallas, la, seattle, orleans, gainesville };

    if (regex_match(buff, optnDig))
    {
        return (buff[0] - '0');
    }
    else if (regex_match(buff, optnCity))
    {
        for (int it = 0; it < options.size(); ++it)
        {
            if (regex_match(buff, options[it]))
                return it + 1;
        }
        cout << "The city you entered was not one of the available options. . . \n If you wish to choose a city outside of the presets, please choose options <8> or <9>." << endl;
        return -1;
    }
    else
    {
        cout << "! Error ! INVALID INPUT\nPlease try again." << endl;
        return -1;
    }
}
//OUTPUT FUNCTIONS, these functions are used to encapsulate output for better clarity in main
void displayHeader() 
{
    for (int i = 0; i < 33; i++)
        cout << " *";
    cout << "\nW e l c o m e   t o    t h e    P o l l o   P r o g r a m m e r s' " << endl;
    cout << "\n";
    cout << "||||| ||||| |||||       ||||  |||   ||||| ||||| ||||| ||||| |||||" << endl;
    cout << "|   | |     | | |       |  || |  || |   |     | |     |       |" << endl;
    cout << "|   | ||||| | | |       ||||  ||||  |   |     | |||   |       |" << endl;
    cout << "|   |     | | | |       |     |  \\  |   | |   | |     |       |" << endl;
    cout << "||||| ||||| |   |       |     |   \\ |||||  |||  ||||| |||||   |" << endl;
    cout << "\n";
    for (int i = 0; i < 33; i++)
        cout << " *";
}
//sets project title in Bridges dependent on city name
void Title(Bridges& proj, string& title)
{
    string fullTitle = "Pollo Graph of: " + title;
    proj.setTitle(fullTitle);
}
// used to informm user on how to enter destination coordinates
void displayDestMenu()
{
    cout << " * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *" << endl;
    cout << "\nP l e a s e   E n t e r   A   D e s t i n a t i o n: \n" << endl;
    cout << "Input TWO values from [0, 100], correspoding to target x and y coordinates on the map:" << endl;
    cout << "Example:\nEnter Input Here:    56, 64.43\n" << endl; 
    cout << " * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n" << endl;
    cout << "Enter Input Here:   ";
}
//displays all options, including presets, exiting the program, and custom city
void displayMainMenu(vector<string>& presets)
{
    displayHeader();
    cout << "\n\nP l e a s e   C h o o s e   a n   O p t i o n: \n\n";
    int it = 0;
    for (it; it < presets.size(); ++it)
    {   
        cout << it + 1 << '.' << " " << presets.at(it) << endl<<endl;

    }
    cout << ++it << '.' << " City Name" << endl<<endl;
    cout << '9' << '.' << " Exit Program.\n" << endl;
    cout << " * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n" << endl;
    cout << "Enter Input Here:   ";
}

// used to inform user on how to enter info for a city name
void displayCityMenu()
{   cout << " * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n" << endl;
    cout << "P l e a s e   E n t e r   A   C i t y: \n";
    cout << "\n! ! ! WARNING ! ! ! \n! Not all cities in the United States are present in the OSM Database ! " << endl;
    cout << "\nPlease refer to http://bridges-data-server-osm.bridgesuncc.org/cities to ensure your intended city is available." << endl;
    cout << "\nInput a city and its state." << endl;
    cout << "\nExample:\nEnter Input Here:    Atlanta, Georgia\n" << endl;
    cout << " * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n" << endl;
    cout << "Enter Input Here:   ";
}

//checks if entered string is valid. If not, returns false
bool isValid(stringstream& input, inputType i)
{
    //Error 1. Not validating input.
    string buff = input.str();
    if (i == inputType::name)
    {
        //regex city_state("[A-Za-z]+(\s[A-Za-z]+)?\s*,*\s*[A-Za-z]+(\s[A-Za-z]+)?\s*");
        regex city_state("[a-zA-Z]+(?: [a-zA-Z]+)?, ?[a-zA-Z]+(?: [a-zA-Z]+)?"); 
        if (regex_match(buff, city_state))
            return true;
        else
        {
            cout << "\n! Error ! \n Invalid input. Please try again." << endl;
            return false;
        }
    }
    else if(i == inputType::rect)
    {
        regex coord("^[+-]?[0-9]{1,4}(\.[0-9]*)?, *[+-]?[0-9]{1,4}(\.[0-9]*)?, *[+-]?[0-9]{1,4}(\.[0-9]*)?, *[+-]?[1-9]{1,4}(\.[0-9]*)?$");
        if (regex_match(buff, coord))
            return true;
        else
        {
            cout << "! Error ! \n Invalid input. Please try again." << endl;
            return false;
        }
    }
    else if(i == inputType::point)
    {
        regex coord("^[+-]?[0-9]{1,4}(\.[0-9]*)?, *[+-]?[0-9]{1,4}(\.[0-9]*)?$");
        if (regex_match(buff, coord))
            return true;
        else
        {
            cout << "! Error ! \n Invalid input. Please try again." << endl;
            return false;
        }
    }
}

int main(int argc, char** argv) {

    string testStr;
    //Part 1: BRIDGES API AND USER API

    int closest;
    double latc, lonc;
    int dest;
    //this vector will be used to pass in strings into Bridges for the preset cities
    vector<string> presetCities = 
    {"Miami, Florida", "New York, New York", "Dallas, Texas", "Los Angeles, California",
        "Seattle, Washington","New Orleans, Louisiana", "Gainesville, Florida" };
   
    string input;
    stringstream ss;
    bool running = true;
    while (running) // allows program to calculate more than once, needs implementation of continue option at end of progrma
    {
        Bridges bridges(0, "DanielT", "1353295928782");
        DataSource ds(&bridges);
        OSMData osm_data;
        bool needIn = true;
        while (needIn) //main menu loop, exits once a choice is made
        {
            displayMainMenu(presetCities);
            input.clear();
            ss.clear();
            getline(cin, input);
            ss.str(input);
            switch (choiceParsing(ss))
            {
            case(-1):
                break;
            case(1): { // 
                osm_data = ds.getOSMData(presetCities[0]);
                Title(bridges, presetCities[0]);
                needIn = false;
                break;
            }
            case(2): {
                osm_data = ds.getOSMData(presetCities[1]);
                Title(bridges, presetCities[1]);
                needIn = false;
                break;
            }
            case(3): {
                osm_data = ds.getOSMData(presetCities[2]);
                Title(bridges, presetCities[2]);
                needIn = false;
                break;
            }
            case(4): {
                osm_data = ds.getOSMData(presetCities[3]);
                Title(bridges, presetCities[3]);
                needIn = false;
                break;
            }
            case(5): {
                osm_data = ds.getOSMData(presetCities[4]);
                Title(bridges, presetCities[4]);
                needIn = false;
                break;
            }
            case(6): {
                osm_data = ds.getOSMData(presetCities[5]);
                Title(bridges, presetCities[5]);
                needIn = false;
                break;
            }
            case(7): {
                osm_data = ds.getOSMData(presetCities[6]);
                Title(bridges, presetCities[6]);
                needIn = false;
                break;
            }
            case(8): {
                bool choose = true;
                while (choose == true)
                {
                    displayCityMenu();
                    input.clear();
                    getline(cin, input);
                    ss.str(input);
                    if (isValid(ss, inputType::name))
                    {
                        try {
                            osm_data = ds.getOSMData(input);
                            
                        }
                        catch (rapidjson_exception) {
                            cout << "This city does Not Exist in the OSM Database, please try another." << endl;
                            continue;
                        }
                    }
                    Title(bridges, input);
                    choose = false;
                }
                needIn = false;
                break;
            }
            case(9): { //exits program
                return 0;
            }
            }
        }
        //initializes all important bridges objects aside from osm_data
        vector<OSMVertex> vertices = osm_data.getVertices();
        vector<OSMEdge> edges = osm_data.getEdges();

        GraphAdjList<int, OSMVertex, double> graph;
        osm_data.getGraph(&graph);
        graph.forceLargeVisualization(true);

        // Part 2: STREETMAP BUILDING
             // Find the closest vertex to the center of your map to be used as the source vertex. 
             // You can get coordinates using OSMVertex.getLatitude() and OSMVertex.getLongitude(). You can color that vertex in the map to see if the calculation is correct.

        int closestCenterIdx;
        double xrange[2] {0, 0};
        double yrange[2]{ 0, 0 };
        //this function is not working
        osm_data.getLatLongRange(xrange, yrange);

        //Part 3: ALGORITHM 
            //finds the shortest path between center and all possible vertices.
        closestCenterIdx = getCenter(osm_data);
        graph.getVertex(closestCenterIdx)->setColor("red");

        unordered_map<int, int> parent;
        vector<int> dists = shortestPath(graph, closestCenterIdx, parent);
        vector<double> destCoords{2,0};
        int dest;
         
        // menu for taking in a coordinate pair for the destination
        bool choose = true;
        while (choose == true)
        {
            displayDestMenu();
            input.clear();
            getline(cin, input);
            stringstream src(input);

            if (isValid(src, inputType::point))
            {
                destCoords = setCoords(src, inputType::point);
                destCoords = convertToLatLong(osm_data, destCoords[0], destCoords[1]);
                dest = getClosestVertex(vertices, destCoords[0], destCoords[1]);
                if (destCoords[0] < xrange[0] || destCoords[0] > xrange[1])
                    cout << "\n! ERROR ! Out of X-range! " << endl;
                if (destCoords[1] < yrange[0] || destCoords[1] > yrange[1])
                    cout << "\n ! ERROR ! Out of Y-range! " << endl;
                else
                    choose = false;
            }
        }     
        //if destination is not accessible
        if (dists[dest] == INT_MAX)
        {
            cout << "\n! WARNING !\n!No possible path to source from this vertex!" << endl;
            cout << "    The following visualization will have no path." << endl;
            styleEnds(graph, closestCenterIdx, dest);
        }
        else{
            cout << "\nPath Found! \nPath Distance: [ " << dists[dest] << " ]" << endl;
            cout << " * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *" << endl;
            styleParent(graph, parent, dest);
        }

        bridges.setDataStructure(&graph);
        bridges.visualize();
        needIn = true;
        while (needIn) {
            cout << "Would you like to find another city? (Y/N)" << endl;
            string in;
            getline(cin, in);
            for (auto& x : in)
                x = toupper(x);
            if (in == "YES" || in == "Y")
            {
                needIn = false;
                system("CLS");
            }
            else if (in == "NO" || in == "N")
            {
                needIn = false;
                running = false;
            }
            else if (in == "(Y/N)"||in == "YN"||in == "Y/N")
            {
                cout << "\n" << endl;
                for (int i = 0; i < 3; i++)
                {
                    this_thread::sleep_for(chrono::nanoseconds(999999999));
                    cout << ".";
                }
                cout << "\n";
                string bruh = "very funny.";
                this_thread::sleep_for(chrono::seconds(1));
                for (auto c : bruh)
                {
                    this_thread::sleep_for(chrono::nanoseconds(100000000));
                    cout << c;
                }
                cout << endl;
            }
            else
                cout << "invalid input! Please try again." << endl;
        }        
    }
    return 0;
}