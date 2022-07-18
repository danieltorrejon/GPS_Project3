#include "Bridges.h"
#include "DataSource.h"
#include "GraphAdjList.h"
#include <iostream>
#include <string>
#include <unordered_map>
#include <fstream>
#include <data_src/OSMData.h>
#include <queue>

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


//used to get the coordinate of the (.25,.25) of the map
void getQuarter(const OSMData& osm_data, double& lat, double& lon) {
    double latr[2];
    double lonr[2];
    osm_data.getLatLongRange(latr, lonr);

    lat = latr[0] + (latr[1] - latr[0]) / 4.;
    lon = lonr[0] + (lonr[1] - lonr[0]) / 4.;
}

//used to get the coordinate of the center of the map
void getCenter(const OSMData& osm_data, double& lat, double& lon) {
    double latr[2];
    double lonr[2];
    osm_data.getLatLongRange(latr, lonr);

    lat = (latr[0] + latr[1]) / 2.;
    lon = (lonr[0] + lonr[1]) / 2.;
}

//actual shortestPath implementation
void shortestPath(const GraphAdjList<int, OSMVertex, double>& gr,
    int source,
    std::unordered_map<int, double>& distance,
    std::unordered_map<int, int>& parent) {
    //TODO
}

//return the vertex the closest to a particular (lat,lon)
int getClosestVertex(const GraphAdjList<int, OSMVertex, double>& graph,
    double lat, double lon) {
    return -1;
}

//style all vertices based on their distance to the root of the shortest path.
//style edges based on whether they sit on a shortest path or not
void styleDistance(GraphAdjList<int, OSMVertex, double> graph,
    const std::unordered_map<int, double>& distance) {
    double maxd = 0.;

    //find max distance (Beware of unreachable vertices with a distance of INFINITY)


    //color vertices based on distances

    //optional: style edges if they are shortest path edges. (Beware of back edges)
}

//style graph based on whether vertices and edges sit on the shortest path between dest and source. (Note that source is not given since all parent pointer chase go there)
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
void styleRoot(GraphAdjList<int, OSMVertex, double>& graph,
    int root) {
    //TODO
}

int main(int argc, char** argv) {


    //create the Bridges object, set credentials
    Bridges bridges(0, "DanielT", "1353295928782");
    bridges.setTitle("Graph : OpenStreet Map Example");

    //Getting Data
    int closest;
    double latc, lonc;
    int dest;

    //TODO: Get data
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

    double xrange[2];
    double yrange[2];
    osm_data.getCartesianCoordsRange(xrange, yrange);

    cout << xrange[0] << " " << xrange[1] << endl;
    cout << yrange[0] << " " << yrange[1] << endl;

    OSMVertex myVertex = vertices[11];
    OSMVertex::OSMVertexID id = myVertex.getVertexID();
    
    
    // Experimentation, WTF is this adjacencyList structure??

    SLelement<Edge<int, double>>* neighbors = graph.getAdjacencyList(0);    // The neighbors of a key (0)
    Edge<int, double> key = neighbors->getValue();
    double edgeData = key.getEdgeData();

    graph.getAdjacencyList(0)->setColor("");
    cout << "Edge weight I think: " << edgeData << endl;

   
    

    //TODO Uncomment for part 2
    // //Getting source vertex (Using center of the map)
    // getCenter(osm_data, latc, lonc);
    // closest = getClosestVertex(graph, latc, lonc);
    // //Getting destination vertex
    // getQuarter(osm_data, latc, lonc);
    // dest = getClosestVertex(graph, latc, lonc);
    // styleRoot(graph, closest);
    // bridges.setDataStructure(&graph);
    // bridges.visualize();


    //TODO Uncomment for part 3.
    // //Running shortest path
    // std::unordered_map<int, double> distance;
    // std::unordered_map<int, int> parent;
    // shortestPath(graph, closest, distance, parent);
    // //Styling based on distance
    // styleDistance(graph, distance);
    // bridges.visualize();

    //TODO Uncomment for part 4
    // //styling based on source-destination path
    // styleParent(graph, distance, parent, dest);
    // bridges.visualize();

    return 0;
}
