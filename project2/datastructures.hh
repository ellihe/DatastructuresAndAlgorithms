// Datastructures.hh
//
// Student name: EH

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <limits>
#include <functional>
#include <exception>
#include <map>
#include <deque>
#include <stack>
#include <queue>

// Types for IDs
using TownID = std::string;
using Name = std::string;
using Road =  std::pair<TownID, TownID>;

// Return values for cases where required thing was not found
TownID const NO_TOWNID = "----------";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
Name const NO_NAME = "!!NO_NAME!!";

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;

    int y = NO_VALUE;
};

// Return value for infinity
double const infinity = std::numeric_limits<double>::infinity();
// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

struct CoordHash
{
    std::size_t operator()(Coord xy) const
    {
        auto hasher = std::hash<int>();
        auto xhash = hasher(xy.x);
        auto yhash = hasher(xy.y);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Type for a distance (in metres)
using Distance = int;

// Return value for cases where Distance is unknown
Distance const NO_DISTANCE = NO_VALUE;

// This exception class is there just so that the user interface can notify
// about operations which are not (yet) implemented
class NotImplemented : public std::exception
{
public:
    NotImplemented() : msg_{} {}
    explicit NotImplemented(std::string const& msg) : msg_{msg + " not implemented"} {}

    virtual const char* what() const noexcept override
    {
        return msg_.c_str();
    }
private:
    std::string msg_;
};


// This is the class you are supposed to implement

class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // In the following estimation of asymptotic complexity, N descripes
    // the amount of towns in datastructure towns_


    // Estimate of performance:
    // Short rationale for estimate:
    unsigned int town_count();

    // Estimate of performance:
    // Short rationale for estimate:
    void clear_all();

    // Estimate of performance:
    // Short rationale for estimate:
    bool add_town(TownID id, Name const& name, Coord coord, int tax);

    // Estimate of performance:
    // Short rationale for estimate:
    Name get_town_name(TownID id);

    // Estimate of performance:
    // Short rationale for estimate:
    Coord get_town_coordinates(TownID id);

    // Estimate of performance:
    // Short rationale for estimate:
    int get_town_tax(TownID id);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<TownID> all_towns();

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<TownID> find_towns(Name const& name);

    // Estimate of performance:
    // Short rationale for estimate:
    bool change_town_name(TownID id, Name const& newname);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<TownID> towns_alphabetically();

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<TownID> towns_distance_increasing();

    // Estimate of performance:
    // Short rationale for estimate:
    TownID min_distance();

    // Estimate of performance:
    // Short rationale for estimate:
    TownID max_distance();

    // Estimate of performance:
    // Short rationale for estimate:
    bool add_vassalship(TownID vassalid, TownID masterid);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<TownID> get_town_vassals(TownID id);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<TownID> taxer_path(TownID id);

    // Non-compulsory phase 1 operations

    // Estimate of performance:
    // Short rationale for estimate:
    bool remove_town(TownID id);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<TownID> towns_nearest(Coord coord);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<TownID> longest_vassal_path(TownID id);

    // Estimate of performance:
    // Short rationale for estimate:
    int total_net_tax(TownID id);


    // Phase 2 operations

    // Estimate of performance: O(N*M), where M is number of neighbours
    // Short rationale for estimate: for loop is linear and clear iside for loop is linear
    void clear_roads();

    // Estimate of performance: O(N*M*K), where M is number of neighbours and K
    //                          length of all_roads vector
    // Short rationale for estimate: linear find method inside forloop which is insise for loop
    std::vector<std::pair<TownID, TownID>> all_roads();

    // Estimate of performance: Average O(1), worst case O(N)
    // Short rationale for estimate: find and at are average constant (worst case linear in container size)
    //                               calculate distance is constant
    bool add_road(TownID town1, TownID town2);

    // Estimate of performance: O(M), where M is number of neighbours
    // Short rationale for estimate: for loop is linear at neighbours' number
    std::vector<TownID> get_roads_from(TownID id);

    // Estimate of performance: average O(N+K), where K is the number of roads,
    //                          Worst case O(N² + K), because of at-function inside it
    // Short rationale for estimate: breadth-first-search is O(N+K), but in worst case at
    //                               function makes it to be O(N²+K)
    std::vector<TownID> any_route(TownID fromid, TownID toid);

    // Non-compulsory phase 2 operations

    // Estimate of performance: average O(1), worst case O(N)
    // Short rationale for estimate: at and erase are average constant
    //                               but in worst case linear in number of towns
    bool remove_road(TownID town1, TownID town2);

    // Estimate of performance: average O(N+K), where K is the number of roads,
    //                          Worst case O(N² + K), because of at-function inside it
    // Short rationale for estimate: estimation is based on estimation of function any route
    std::vector<TownID> least_towns_route(TownID fromid, TownID toid);

    // Estimate of performance: average O(N+K), where K is the number of roads,
    //                          Worst case O(N² + K²), because of at-function inside it
    // Short rationale for estimate: estimation is based on dfs functions estimation
    std::vector<TownID> road_cycle_route(TownID startid);

    // Estimate of performance: average O((N+K)log(N+K)), worst case O((N²+K)log(N²+K)),
    //                          where K is the number of roads
    // Short rationale for estimate: estimation is based on dijkstra functions estimation
    std::vector<TownID> shortest_route(TownID fromid, TownID toid);

    // Estimate of performance: O(K*log(K)), worst case O(K*NZX)
    // Short rationale for estimate: std::sort is O(K*log(K)),
    //                               for loops are average O(K), but worst case O(K*N)
    //                               where K is number of roads
    Distance trim_road_network();

private:


    enum Colour {WHITE, GRAY, BLACK};
    // Add stuff needed for your class implementation here
    struct Town {
        TownID id_;
        Name name_;
        Coord coord_;
        int tax_;
        std::vector<TownID> vassals_;
        TownID masterid_;

        // Data data for roads
        TownID prev_node_ = "";
        double distance_ = infinity;
        Colour colour = WHITE;
        std::unordered_map<TownID, int> to_neighbours_;

        // Data for loop search (dfs)
        TownID loop_node = "";

        // Data for MST
        TownID parent_ = id_;
        int rank_ = 0;
    };

    std::unordered_map<TownID, Town> towns_;

    // Estimate of performance: O(1)
    // Short rationale for estimate: Amount of data does not effect to
    //                               calculate time of distance of two coordinated
    double calculate_distance(Coord x, Coord y);

    // Estimate of performance: Average O(N), worst case O(N squared)
    // Short rationale for estimate: find, end, push_back are O(1)
    //                               function taxers_recursion will be called N times,
    //                               and at (average constant, worst case linear) is also called N -times
    std::vector<TownID> taxers_rekursion(TownID id, std::vector<TownID> taxers);


    // Estimate of performance: O(N squared)
    // Short rationale for estimate: Recursion inside for loop makes function to be N squared
    int total_net_tax_recursion(TownID id);

    // Estimate of performance: average O(N+K), where K is the number of roads,
    //                          Worst case O(N² + K), because of at-function inside it
    // Short rationale for estimate: breadth-first-search is O(N+K), but in worst case at
    //                               function makes it to be O(N²+K)
    void bfs_for_any_route(TownID id);

    // Estimate of performance: average O(N+K), where K is the number of roads,
    //                          Worst case O(N² + K²), because of at-function inside it
    // Short rationale for estimate: breadth-first-search is O(N+K), but in worst case at
    //                               function makes it to be O(N²+K)
    Road dfs_for_road_cycle_roat(TownID id);

    // Estimate of performance: average O((N+K)log(N+K)), worst case O((N²+K)log(N²+K)),
    //                          where K is the number of roads
    // Short rationale for estimate: dijkstra is O((N+K)log(N+K)), but in the worst case
    //                               at function makes is to be O((N²+K)log(N²+K))
    void dijkstra_for_shortest_route(TownID);

    // Estimate of performance: average O(M), worst case O(M²), where M is the amount of parents
    //                          (the depth of subtree)
    // Short rationale for estimate: rekursive function will be called M times, and at function
    //                               inside recursion makes it to be squared in worst case
    TownID find_root(TownID town);
};

#endif // DATASTRUCTURES_HH
