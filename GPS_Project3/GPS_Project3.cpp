// STD Library
#include <iostream>
#include <string>
#include <unordered_map>
#include <fstream>
#include <queue>
#include <iomanip>
#include <regex>
// Bridges Repository
#include "Bridges.h" // This class contains methods to connect and transmit a user's data structure representation to the Bridges server.
#include "DataSource.h" // This class provides an API to various data sources used in BRIDGES.
#include "GraphAdjList.h" // This class provides methods to represent adjacency list based graphs.
#include <data_src/OSMData.h> // Class that holds Open Street Map data, from https://openstreetmap.org


//Read through documentation again.!
using namespace std;
using namespace bridges;

enum inputType
{
    coords,
    name,
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

// ======== DIJKSTRA'S SHORTEST PATH IMPLEMENTATION ========= O( (V+E)log(V) ) time
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
    int dest) {
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
    regex optnCity("[a-zA-Z]+,?\s?([a-zA-Z]+)?");
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
//in progress, converts string "43,54,234,54" into vector of ints
vector<double> setCoords(istringstream& src)
{
    vector<double> result;
    string x;
    for(int i = 0; i < 3; ++i)
    { 
        getline(src, x, ',');
        result[i] = stod(x);
    }
    getline(src, x);
    result[3] = stod(x);
        
    if (result.at(0) > result.at(2))
        swap(result[0], result[2]);
    if (result[1] > result[3])
        swap(result[1], result[3]);
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
        cout << "Coordinates: <min latitude, min longitude, max latitude, max longitude>" << endl;
}
// manages menu for option 8 and 9. enter data


//checks if entered string is valid. If not, returns false.
bool isValid(istringstream& input, inputType i)
{
    if (i == inputType::name)
    {
        regex city_state("^[A-Za-z]+(\s[A-Za-z]+)*\s*,*\s*[A-Za-z]+(\s[A-Za-z]+)*\s*$");
        if (regex_match(input.str(), city_state))
            return true;
        else
        {
            cout << "! Error ! \n Invalid input. Please try again." << endl;
        }
    }
    else
    {
        regex coordFormat("^([+\-]?[0-9]{1,3}(\.[0-9]{0,2})?)(,\s*([+\-]?[0-9]{1,3}(\.[0-9]{0,2})?)){3}");
        if (regex_match(input.str(), coordFormat))
            return true;
        else
           cout << "! Error ! \n Invalid input. Please try again." << endl;
    }
}
   //OSMData osm_data = ds.getOSMData(35.28, -80.8, 35.34, -80.7, "tertiary"); //UNCC Campus
    //OSMData osm_data = ds.getOSMData(39.85, -83.14, 40.12, -82.85, "secondary"); //Columbus, OH
    //OSMData osm_data = ds.getOSMData(39.121, -77.055, 39.208, -76.805); //Baltimore, MD
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

    vector<string> presetCities = {"Miami, Florida", "New York City, New York",
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
            case(1): {
                osm_data = ds.getOSMData(presetCities[0]);
                needIn = false;
                break;
            }
            case(2): {
                osm_data = ds.getOSMData(presetCities[1]);
                needIn = false;
                break;
            }
            case(3): {
                osm_data = ds.getOSMData(presetCities[2]);
                needIn = false;
                break;
            }
            case(4): {
                osm_data = ds.getOSMData(presetCities[3]);
                needIn = false;
                break;
            }
            case(5): {
                osm_data = ds.getOSMData(presetCities[4]);
                needIn = false;
                break;
            }
            case(6): {
                osm_data = ds.getOSMData(presetCities[5]);
                needIn = false;
                break;
            }
            case(7): {
                osm_data = ds.getOSMData(presetCities[6]);
                needIn = false;
                break;
            }
            case(8): {
                bool choose = true;
                while (choose == true)
                {
                    cityMenu(inputType::name);
                    input.clear();
                    cin >> input;
                    ss.str(input);
                    if (isValid(ss, inputType::name))
                    {
                        try {
                            osm_data = ds.getOSMData(input);
                        }
                        catch (exception) {
                            cout << "This city does Not Exist in the OSM Database, please try another." << endl;
                        }
                    }
                    else
                        continue;
                }
                needIn = false;
                break;
            }
            case(9): {
                string city;
                vector<double> edges;
                bool choose = true;
                while (choose == true)
                {
                    cityMenu(inputType::coords);
                    input.clear();
                    cin >> input;
                    ss.str(input);
                    if (isValid(ss, inputType::coords))
                    {
                        edges = setCoords(ss);
                        try {
                            osm_data = ds.getOSMData(edges[0],edges[1],edges[2],edges[3]);
                        }
                        catch (exception) {
                            cout << "This city does Not Exist in the OSM Database, please try another." << endl;
                        }
                    }
                    else
                        continue;
                }
                needIn = false;
                break;
            }
            case(0): { //exits program
                return 0;
            }
            }
        }
        vector<OSMVertex> vertices = osm_data.getVertices();
        vector<OSMEdge> edges = osm_data.getEdges();

        GraphAdjList<int, OSMVertex, double> graph; //GET THE ADJACENCY LIST
        osm_data.getGraph(&graph);
        graph.forceLargeVisualization(true);

        //initializes xrange and yrange as 2-element arrays, holding x-min and y-min at [0] and x and y maxes at [1]
        double xrange[2];
        double yrange[2];
        osm_data.getCartesianCoordsRange(xrange, yrange);

        cout << "X range: " << xrange[0] << " to " << xrange[1] << endl;
        cout << "Y range: " << yrange[0] << " to " << yrange[1] << endl;

   
// Part 2: STREETMAP BUILDING
        // Find the closest vertex to the center of your map to be used as the source vertex. 
        // You can get coordinates using OSMVertex.getLatitude() and OSMVertex.getLongitude(). You can color that vertex in the map to see if the calculation is correct.

    int closestCenterIdx;
    int dest;

    closestCenterIdx = getCenter(osm_data);
    graph.getVertex(closestCenterIdx)->setColor("red");

    unordered_map<int, int> parent;
    shortestPath(graph, closestCenterIdx, parent);



//Part 3: ALGORITHM
        

    dest = 4542;    // This is a RANDOM vertex index that I found within the parent unordered_map
    
    // Utilize this function to get the vertex for inputted latitude and longitude coordinates. Check ranges first!
    // dest = getClosestVertex(graph, latc, lonc);


    int child = dest;
    int successor = parent[dest];

    // Here is how I drew the path
    while (successor != -1)
    {
        graph.getVertex(child)->setColor("red");
        child = successor;

        successor = parent[child];
    }

    bridges.setDataStructure(&graph);
    bridges.visualize();

//Part 3: ALGORITHM - Daniel
    // Computing distance from a source to all vertices: Shortest Path Algorithm, Djikstra  
    //<int_child>,int_parent>
   
    //shortestPath(graph, closestCenterIdx, parent);
    //iterate through parent map, for each index go to parent, access vertex specifically and edge and change color
//Part 4: Destination - Adrian/Daniel
    // 1. Take in an input for destination
    // 2. Validate input, OR output vector contents as options to choose from
    // 3. Identify path between source and destination :Graph Algorithms, Pointer Chasing
    //      a. How we do this is entirely dependent on what we get from Part 3, the adjacency list, the map, etc.
     //adjacency list need not be worked with
//Part 5: OUTPUT - Adrian
    /*
    * //edge.setcolor
    * //vertex.setcolor
    * d[v] is a vector of distances
    * p[v] parent map 
     Styling based on source-destination path
     Color the map based on distance from source vertex

     styleParent(graph, distance, parent, dest);
     bridges.visualize();
    
    BLOCK OFFERED BY PPT GUIDE
   
    */  
    /*
    > This will be at the end of the program to offer an option to restart the process.
    > Not super important, can remove if it's troublesome. (Memory Leaks)
    
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


