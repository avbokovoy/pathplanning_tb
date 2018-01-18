#ifndef STRUCTS_H
#define STRUCTS_H
#include <vector>

struct Node
{
    int     i, j;
    double  F, g, H;
    Node    *parent;

    bool operator== (const Node &other) const {
        return i == other.i && j == other.j;
    }
};

struct SearchResult
{
        bool pathfound;
        float pathlength;
        std::vector<Node> hppath;
        SearchResult()
        {
            pathfound = false;
            pathlength = 0;
        }
};


struct sPosition
{
    float x;
    float y;
    float z;
};

struct sOrientation
{
    float x;
    float y;
    float z;
    float w;
};

struct sOrigin
{
    sPosition position;
    sOrientation orientation;
};

struct sInfo
{
    int width;
    int height;
    sOrigin origin;
    float resolution;
};

struct OccupancyGrid
{
    sInfo info;
    //!!! Replace back, if something goes wrong
    //int data[];
    int* data;
};

struct sPose
{
    sPosition position;
    sOrientation orientation;
};

struct goalPose
{
    sPose pose;
};

#endif // STRUCTS_H
