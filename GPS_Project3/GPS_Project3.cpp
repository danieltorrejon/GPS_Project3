// STD Library
#include <iostream>
#include <string>
#include <unordered_map>
#include <fstream>
#include <queue>
#include <iomanip>
// Bridges classes
#include "Bridges.h" // This class contains methods to connect and transmit a user's data structure representation to the Bridges server.
#include "DataSource.h" // This class provides an API to various data sources used in BRIDGES.
#include "GraphAdjList.h" // This class provides methods to represent adjacency list based graphs.
#include <data_src/OSMData.h> // Class that holds Open Street Map data, from https://openstreetmap.org

using namespace std;
using namespace bridges;

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
int getCenter(const OSMData& osm_data) {
    double latr[2];
    double lonr[2];
    osm_data.getLatLongRange(latr, lonr);

    double lat = (latr[0] + latr[1]) / 2.;
    double lon = (lonr[0] + lonr[1]) / 2.;
    vector<OSMVertex> vertices = osm_data.getVertices();

    double latV = 0;
    double lonV = 0;
    bool found = false;
    int index = -1;
    double min = INT_MAX;

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

//actual shortestPath implementation, Part 3
void shortestPath(const GraphAdjList<int, OSMVertex, double>& gr,
    int source,
    unordered_map<int, double>& distance,
    unordered_map<int, int>& parent)
{   
    // A heap of pairs, where the pair stores <Distance, Index> as <int, int>
    

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>> > pq;    // min heap
    vector<int> d(N, INT_MAX);

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

bool inputParsing(istringstream& input)
{
    return true;
}
void outputAPI()
{
    cout << "Welcome to the OSM Project" << endl;
    cout << "Please Choose an Input Method: \n1. Name of City\n2. Cartesian Coordinates" << endl;
}
int main(int argc, char** argv) {

    //create the Bridges object, set credentials
    Bridges bridges(0, "DanielT", "1353295928782");
    bridges.setTitle("Graph : OpenStreet Map Example");

//Part 1: BRIDGES API AND USER API
    // Get data from an API into the program
    // Build a graph for your city using the edge data you just obtained (or use Bridges builtin graph build OSMData.getGraph)
    // Render that map using Bridges
    int closest;
    double latc, lonc;
    int dest;

    /* Get Data for an Area
        There are two ways for a user to request data:
            1. City Name : "Charlotte, North Carolina"
            2. Coordinates: "39.121, -77.055, 39.208, -76.805"

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
    // Adrian will cover handling User Input and Edge Cases!
   
    DataSource ds(&bridges);
    OSMData osm_data = ds.getOSMData("Charlotte, North Carolina");
    //OSMData osm_data = ds.getOSMData(35.28, -80.8, 35.34, -80.7, "tertiary"); //UNCC Campus
    //OSMData osm_data = ds.getOSMData(39.85, -83.14, 40.12, -82.85, "secondary"); //Columbus, OH
    //OSMData osm_data = ds.getOSMData(39.121, -77.055, 39.208, -76.805); //Baltimore, MD

    vector<OSMVertex> vertices = osm_data.getVertices();
    vector<OSMEdge> edges = osm_data.getEdges();

    GraphAdjList<int, OSMVertex, double> graph;
    osm_data.getGraph(&graph);
    graph.forceLargeVisualization(true);

    //initializes xrange and yrange as 2-element arrays, holding x-min and y-min at [0] and x and y maxes at [1]
    double xrange[2]; 
    double yrange[2];
    osm_data.getCartesianCoordsRange(xrange, yrange);

    cout << "X-Range:\n" << "x-min: [" << xrange[0] <<"] " << "x-max: [" << xrange[1] << "]"<< endl;
    cout << "Y-Range: \n" << "y-min: [" << yrange[0] << "] " << "y-max: [" << yrange[1] << "]" << endl;

    OSMVertex myVertex = vertices[11];
    OSMVertex::OSMVertexID id = myVertex.getVertexID();
    
    // Experimentation, WTF is this adjacencyList structure??
    cout << endl;
    cout << "======= STUFF IM DOING =======" << endl << endl;

    int num = 342;
    
    // This is how you iterate through adjacent elements
    // Just a singular path of the many that can be taken
    // sort of
    for (int i = 0; i < 0; i++)
    {
        SLelement<Edge<int, double>>* neighbors = graph.getAdjacencyList(num);
        Edge<int, double> edge = neighbors->getValue();

        graph.getVertex(num)->setColor("red");
        num = edge.to();
        cout << "Vertex: " << num << endl;
    }

    int index = getCenter(osm_data);
    
    graph.getVertex(index)->setColor("red");


   
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
    bridges.setDataStructure(&graph);
    bridges.visualize();

//Part 3: ALGORITHM
        // Computing distance from a source to all vertices: Shortest Path Algorithm, Djikstra
        // Identifying path between source and destination :Graph Algorithms, Pointer Chasing
    //TODO Uncomment for part 3.
  
    unordered_map<int, double> distance;
    unordered_map<int, int> parent;
    shortestPath(graph, closest, distance, parent);
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

    return 0;
}
