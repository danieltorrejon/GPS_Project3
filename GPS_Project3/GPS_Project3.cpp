// STD Library
#pragma once
#include <iostream>
#include <string>
#include <unordered_map>
#include <fstream>
#include <queue>
#include <iomanip>
#include <regex>
// Bridges classes
#include "Bridges.h" // This class contains methods to connect and transmit a user's data structure representation to the Bridges server.
#include "DataSource.h" // This class provides an API to various data sources used in BRIDGES.
#include "GraphAdjList.h" // This class provides methods to represent adjacency list based graphs.
#include <data_src/OSMData.h> // Class that holds Open Street Map data, from https://openstreetmap.org

using namespace std;
using namespace bridges;

enum inputType
{
    coords,
    name
};
//function used to return the relative distance between doubles.
//useful to compare for equality
double RelDif(double a, double b) {
    double c = std::abs(a);
    double d = std::abs(b);

    d = (std::max)(c, d);

    return d == 0.0 ? 0.0 : std::abs(a - b) / d;
}

//used to get the coordinate of the (.25,.25) of the map, Part 2
void getQuarter(const OSMData& osm_data, double& lat, double& lon) {
    double latr[2];
    double lonr[2];
    osm_data.getLatLongRange(latr, lonr);

    lat = latr[0] + (latr[1] - latr[0]) / 4.;
    lon = lonr[0] + (lonr[1] - lonr[0]) / 4.;
}

//used to get the coordinate of the center of the map, Part 2
void getCenter(const OSMData& osm_data, double& lat, double& lon) {
    double latr[2];
    double lonr[2];
    osm_data.getLatLongRange(latr, lonr);

    lat = (latr[0] + latr[1]) / 2.;
    lon = (lonr[0] + lonr[1]) / 2.;
}

//actual shortestPath implementation, Part 3
void shortestPath(const GraphAdjList<int, OSMVertex, double>& gr,
    int source,
    std::unordered_map<int, double>& distance,
    std::unordered_map<int, int>& parent) {
    //TODO
}

//return the vertex the closest to a particular (lat,lon), Part 3
int getClosestVertex(const GraphAdjList<int, OSMVertex, double>& graph,
    double lat, double lon) {
    return -1;
}

//style all vertices based on their distance to the root of the shortest path. // Part 4
//style edges based on whether they sit on a shortest path or not
void styleDistance(GraphAdjList<int, OSMVertex, double> graph,
    const std::unordered_map<int, double>& distance) {
    double maxd = 0.;

    //find max distance (Beware of unreachable vertices with a distance of INFINITY)


    //color vertices based on distances

    //optional: style edges if they are shortest path edges. (Beware of back edges)
}

//style graph based on whether vertices and edges sit on the shortest path between dest and source. (Note that source is not given since all parent pointer chase go there) // Part 4
void styleParent(GraphAdjList<int, OSMVertex, double> graph,
    const std::unordered_map<int, double>& distance,
    const std::unordered_map<int, int>& parent,
    int dest
) {
    //TODO

    //set all edges to transparent

    //set all vertices to transparent


    //for each edge on the SP from source to dest
}

//change the style of the root of the shortest path
void styleRoot(GraphAdjList<int, OSMVertex, double>& graph, int root) {
    //TODO
}

int choiceParsing(istringstream& input)
{
    string buff;
    input >> buff;
    regex optnDig("[0-9]\.?"); //matches a digit from 1-9 and then a possible . after
    regex optnCity("[[:alpha:]]+,?\s?([[:alpha:]]+)?");
    regex miami("/miami,?\s?(florida)?/i");
    regex dallas("/dallas,?\s?(texas)?/i");
    regex chicago("/chicago,?\s?(illinois)?/i");
    regex seattle("/seattle,?\s?(washington)?/i");
    regex orleans("/new\sorleans,?\s?(louisiana)?/i");
    regex gainesville("/gainesville,?\s?(florida)?/i");
    regex newyork("/new york(city)?,?\s?(new york)/i?");
    vector<regex> options
    { miami, newyork, dallas, chicago, seattle, orleans, gainesville };
    if(regex_match(buff, optnDig))
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
 // implement maybe, solely for aesthetic purposes
void header() //outputs pretty graphic
{

}
//outputs main menu, showing options
void mainMenu(vector<string>& presets)
{
    cout << "Welcome to the OSM Project" << endl;
    {
        cout << "Please Choose an Option: \n\n";
        cout << "L a r g e   U. S.   C i t i e s\n";
        int it = 0;
        for (it; it < presets.size(); ++it)
            cout << it + 1 << '.' << presets.at(it) << endl;
        cout << "User Input" << endl;
        cout << ++it << '.' << "City Name" << endl;
        cout << ++it << '.' << "Coordinates" << endl;
        cout << '0' << '.' << "Exit Program." << endl;
    }
}
//in progress, converts string "43,54,234,54" into vector of ints
vector<int> setCoords(string& src)
{
    vector<int> result;
    //use getline
    return result;
}
// outputs info for user input
void cityMenu(inputType i)
{
    
    cout << "!!!WARNING!!!\n Not all cities in the United States are present in the OSM Database. " << endl;
    cout << "Please refer to http://bridges-data-server-osm.bridgesuncc.org/cities to ensure your intended city is available." << endl;
    cout << "Enter your input in one of the following ways:  ";
    if(i == name)
        cout << "Name:  <city name>, <state>" << endl; 
    else if (i == coords)
        cout << "Coordinates: <,>" << endl;
    cout << "Input '0' to return to the Main Menu." << endl;
}
// manages menu for option 8 and 9. enter data
bool enterCityData(string& input)
{   
    bool choose = true;
    while (choose == true)
    {
        cityMenu(name);
        input.clear();
        cin >> input;
        if (input == "0")
            return false;
        else if (isValid(input,name))
            return true;
    }
}
bool enterCityData(string& input, vector<int> coord) 
{
    bool choose = true;
    while (choose == true)
    {
        cityMenu(coords);
        input.clear();
        cin >> input;
        if (input == "0")
            return false;
        else if (isValid(input, coords))
        {
            coord = setCoords(input);
            return true;
        }
    }
}
//checks if entered string is valid. If not, returns false.
bool isValid(string& input, inputType i)
{

 
    /*
        Invalid city name edge cases:
            > incorrect formatting
                e.g. AlbanyNewYork
            > city does not exist
                e.g. Atlantis, New York
            > found in Bridges, but state is not specified
                ( only an edge case if there is a city with the same name in a different state)
                    eg Albany, Georgia vs Albany, New York
        Invalid Coordinates edge cases:
            > too many coordinates:
                e.g. 32, 43, 54, 67, 754, 35, 53252, 462
            >
    */

   //OSMData osm_data = ds.getOSMData(35.28, -80.8, 35.34, -80.7, "tertiary"); //UNCC Campus
    //OSMData osm_data = ds.getOSMData(39.85, -83.14, 40.12, -82.85, "secondary"); //Columbus, OH
    //OSMData osm_data = ds.getOSMData(39.121, -77.055, 39.208, -76.805); //Baltimore, MD
    return true;
}
int main(int argc, char** argv) {

    //create the Bridges object, set credentials
    Bridges bridges(0, "DanielT", "1353295928782");
    bridges.setTitle("Graph : OpenStreet Map Example");
    
    
    /*
    * potential regex for y/n choices in API
    * regex yes("((/y/i)(es)?)+");
    * regex no("((/n/i)(o)?)+");
    */
    
    //Part 1: BRIDGES API AND USER API

    int closest;
    double latc, lonc;
    int dest;

    vector<string> presetCities ={ "buffer", "Miami, Florida", "New York City, New York",
    "Dallas, Texas", "Chicago, Illinois", "Seattle, Washington","New Orleans, Louisiana", "Gainesville, Florida" };
    string input;
    DataSource ds(&bridges);
    OSMData osm_data;
    bool running = true;
    while (running) // allows program to calculate more than once, needs implementation of continue option at end of progrma
    {
        bool needIn = true;
        while (needIn) //main menu loop, exits once a choice is made
        {
            mainMenu(presetCities);
            getline(cin, input);
            istringstream ss(input);
            switch (choiceParsing(ss))
            {
                case(-1):
                    break;
                case(0):{
                    osm_data = ds.getOSMData(presetCities[0]);
                    needIn = false;
                    break;
                }
                case(1): {
                    osm_data = ds.getOSMData(presetCities[1]);
                    needIn = false;
                    break;
                }
                case(2):{
                    osm_data = ds.getOSMData(presetCities[2]);
                    needIn = false;
                    break;
                }
                case(3):{
                    osm_data = ds.getOSMData(presetCities[3]);
                    needIn = false;
                    break;
                }
                case(4):{
                    osm_data = ds.getOSMData(presetCities[4]);
                    needIn = false;
                    break;
                }
                case(5):{
                    osm_data = ds.getOSMData(presetCities[5]);
                    needIn = false;
                    break;
                }
                case(6):{
                    osm_data = ds.getOSMData(presetCities[6]);
                    needIn = false;
                    break;
                }
                case(7):{
                    string city;
                    if (enterCityData(city) == true){
                        osm_data = ds.getOSMData(city);
                        needIn = false;
                    }
                    break;
                }
                case(8):{ 
                    string city;
                    vector<int> coords;
                    if (enterCityData(city,coords) == true){
                        osm_data = ds.getOSMData(coords[0],coords[1],coords[2],coords[3]);
                        needIn = false;
                    }
                    break;
                }
                case(9):{ //exits program
                    return 0;
                }
            }
        }

        vector<OSMVertex> vertices = osm_data.getVertices();
        vector<OSMEdge> edges = osm_data.getEdges();

        GraphAdjList<int, OSMVertex, double> graph;
        osm_data.getGraph(&graph);
        graph.forceLargeVisualization(true);

        //initializes xrange and yrange as 2-element arrays, holding x-min and y-min at [0] and x and y maxes at [1]
        double xrange[2];
        double yrange[2];
        osm_data.getCartesianCoordsRange(xrange, yrange);

        cout << "X-Range:\n" << "x-min: [" << xrange[0] << "] " << "x-max: [" << xrange[1] << "]" << endl;
        cout << "Y-Range: \n" << "y-min: [" << yrange[0] << "] " << "y-max: [" << yrange[1] << "]" << endl;

        OSMVertex myVertex = vertices[11];
        OSMVertex::OSMVertexID id = myVertex.getVertexID();

        // Experimentation, WTF is this adjacencyList structure??

        SLelement<Edge<int, double>>* neighbors = graph.getAdjacencyList(0);    // The neighbors of a key (0)
        Edge<int, double> key = neighbors->getValue();
        double edgeData = key.getEdgeData();

        graph.getAdjacencyList(0)->setColor("");
        cout << "Edge weight I think: " << edgeData << endl;


        // Part 2: STREETMAP BUILDING
                // Find the closest vertex to the center of your map to be used as the source vertex. 
                // You can get coordinates using OSMVertex.getLatitude() and OSMVertex.getLongitude(). You can color that vertex in the map to see if the calculation is correct.
            //TODO Uncomment for part 2

            // getCenter(osm_data, latc, lonc);
            // closest = getClosestVertex(graph, latc, lonc);
            // Getting destination vertex
            // getQuarter(osm_data, latc, lonc);
            // dest = getClosestVertex(graph, latc, lonc);
            // styleRoot(graph, closest);
            // bridges.setDataStructure(&graph);
            // bridges.visualize();

        //Part 3: ALGORITHM
                // Computing distance from a source to all vertices: Shortest Path Algorithm, Djikstra
                // Identifying path between source and destination :Graph Algorithms, Pointer Chasing
            //TODO Uncomment for part 3.

            // std::unordered_map<int, double> distance;
            // std::unordered_map<int, int> parent;
            // shortestPath(graph, closest, distance, parent);
            // //Styling based on distance
            // styleDistance(graph, distance);
            // bridges.visualize();

        //Part 4: OUTPUT
                // styling based on source-destination path
                // Color the map based on distance from source vertex
            //TODO Uncomment for part 4

        ElementVisualizer* styler;
        // styleParent(graph, distance, parent, dest);
        // bridges.visualize();
        /*
        needIn = true;
        while(needIn)
        cout << "Would you like to find another city? (Y/N)" << endl;
        string in;
        cin >> in;
        if(regex_search(in, yes))

        else if(regex_search(in, no))

        else
        */
    }
    return 0;
}

