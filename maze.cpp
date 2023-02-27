#include <vector>
#include <set>
#include <algorithm>

#include <cstdio>
#include <cstdlib>
#include <sys/time.h>

extern "C" {
#include <pbm.h>
}

struct Adjacency {
  Adjacency() : to_east(false), to_south(false) {}

  bool  to_east : 1;
  bool  to_south : 1;
};

using Connections = std::vector<std::vector<Adjacency>>;
using Pos = std::pair<int,int>; // y,x
using Path = std::vector<Pos>;
using Mask = std::set<Pos>;

// Create a connectivity matrix represting a random
// Width*Heigh maze with exactly one path connecting
// (sx,sy) to (fx,fy).  The mask parameter optionally
// specifies a subset of cells whose walls will not be
// removed (e.g. for drawing pictures in the maze).
// Return result in out-parameter 'adj_out'
void Generate(Connections &adj_out, int Width, int Height,
              int sx, int sy, int fx, int fy,
              const Mask &mask) {
  // Initialize the graph: all walls are up!
  adj_out.clear();
  adj_out.resize(Height);
  for(auto &row : adj_out)
    row.resize(Width);

  // Track connected components via union-find
  class EqClass {
    EqClass *rep;

    EqClass *find_rep() {
      if( rep != this )
        rep = rep->find_rep();
      return rep;
    }

  public:
    EqClass() : rep(this) {}

    void merge(EqClass &other) {
      EqClass *r1 = this->find_rep(),
              *r2 = other.find_rep();
      if( r1 != r2 )
        r2->rep = r1;
    }

    bool operator==(EqClass &other) {
      return this->find_rep() == other.find_rep();
    }
    bool operator!=(EqClass &other) {
      return !( *this == other );
    }
  };

  // Map each position on the maze to its component
  std::vector<std::vector<EqClass>> pos2ec;
  pos2ec.resize(Height);
  for(auto &row : pos2ec)
    row.resize(Width);

  // Repeatedly choose a random wall and
  // knock it down if it merges two components
  int total_iterations=0,
      productive_iterations=0;
  for(;; ++total_iterations) {
    if( 0 == (rand() % 2) ) {
      // East
      int x = rand() % (Width-1);
      int y = rand() % Height;

      if( mask.count( Pos(y,x) ) )
        continue;

      if( adj_out[y][x].to_east )
        continue; // No change

      EqClass &ec1 = pos2ec[y][x];
      EqClass &ec2 = pos2ec[y][x+1];
      if( ec1 == ec2 )
        continue; // No change

      adj_out[y][x].to_east = true;
      ec1.merge(ec2);
    }
    else {
      // South
      int x = rand() % Width;
      int y = rand() % (Height-1);

      if( mask.count( Pos(y,x) ) )
        continue;

      if( adj_out[y][x].to_south )
        continue; // No change

      EqClass &ec1 = pos2ec[y][x];
      EqClass &ec2 = pos2ec[y+1][x];
      if( ec1 == ec2 )
        continue; // No change

      adj_out[y][x].to_south = true;
      ec1.merge(ec2);
    }

    // Only test if start,finish are connected
    // on iterations that change connectivity
    ++productive_iterations;
    if( pos2ec[sy][sx] == pos2ec[fy][fx] )
      break;
  }

  // A performance statistic
  printf("%.2f%% productive (%d/%d)\n",
    100 * productive_iterations / (float)total_iterations,
    productive_iterations,total_iterations);
}

// Render the connectivity matrix 'adj' to stdout as
// a Width*Height maze.  Mark start position (sx,sy)
// with an 'S' and finish position (fx,fy) with an 'E'.
// If a path is provided, mark points along that path
// with breadcrumbs '.'
void Draw(const Connections &adj, const Path &path,
          const Mask &mask,
          int Width, int Height,
          int sx, int sy, int fx, int fy) {
  for(int i=0; i<2*Width+1; ++i)
    printf("█");
  printf("\n");

  for(int y=0; y<Height; ++y) {
    printf("█");
    for(int x=0; x<Width; ++x) {
      // Print the cell
      if( x==sx && y==sy )
        printf("S");
      else if( x==fx && y==fy )
        printf("E");
      else if( std::find(path.begin(),path.end(), Pos(y,x))
               != path.end() )
        printf(".");
      else if( mask.count( Pos(y,x) ) )
        printf("█");
      else
        printf(" ");

      // Print the wall, if present
      if( x+1 >= Width || !adj[y][x].to_east )
        printf("█");
      else
        printf(" ");
    }
    printf("\n█");
    for(int x=0; x<Width; ++x) {
      if( y+1>=Height || !adj[y][x].to_south )
        printf("██");
      else
        printf(" █");
    }
    printf("\n");
  }
}

// Attempt to solve the maze represented by connectivity
// matrix 'adj' of size Width*Height starting at (sx,sy)
// and leading to (fx,fy).  If successful, return true
// and store the path in output parameter 'path_out'.
// Otherwise, return false.
bool Solve(const Connections &adj, int Width, int Height,
           int sx, int sy, int fx, int fy, Path &path_out) {
  std::vector<Path> fringe;
  fringe.push_back( {} );
  fringe.back().emplace_back( sy, sx );

  while( !fringe.empty() ) {
    Path some_path;
    some_path.swap( fringe.back() );
    fringe.pop_back();

    Pos &p1 = some_path.back();

    if( p1.first==fy && p1.second==fx ) {
      path_out.swap(some_path);
      return true;
    }

    auto step = [&fringe](const Path &some_path, const Pos p2) {
      if( std::find(some_path.rbegin(), some_path.rend(), p2)
          == some_path.rend() ) { // reverse search because locality
        fringe.push_back(some_path);
        fringe.back().push_back(p2);
      }
    };

    Pos p2 = {p1.first-1, p1.second}; // North
    if( p2.first >= 0 && adj[p2.first][p2.second].to_south )
      step(some_path,p2);
    p2 = {p1.first, p1.second+1 }; // East
    if( p2.second < Width && adj[p1.first][p1.second].to_east )
      step(some_path,p2);
    p2 = {p1.first+1, p1.second}; // South
    if( p2.first < Height && adj[p1.first][p1.second].to_south )
      step(some_path,p2);
    p2 = {p1.first, p1.second-1}; // West
    if( p2.second >= 0 && adj[p2.first][p2.second].to_east )
      step(some_path,p2);
  }

  return false; // No path found
}

// Usage: ./maze.exe [Width Height] [breadcrumbs] [seed]
int main(int argc, char *argv[]) {

  Connections conns;

  int Width=50;
  int Height=50;
  int breadcrumbs = 1;
  const char *mask_fn = nullptr;

  struct timeval now;
  gettimeofday(&now,nullptr);
  int seed = (int) now.tv_usec;

  if( argc > 2 ) {
    Width = atoi(argv[1]);
    Height = atoi(argv[2]);
  }
  if( argc > 3 )
    breadcrumbs =!! atoi(argv[3]);
  if( argc > 4 ) {
    int s = atoi(argv[4]);
    if( s != -1 )
      seed = s;
  }
  if( argc > 5 )
    mask_fn = argv[5];

  srand(seed);

  Mask mask;

  // Try to load the mask image if sizing compatible.
  if( mask_fn )
    if( FILE *fin = pm_openr(mask_fn) ) {
      int cols, rows;
      if( bit **b = pbm_readpbm(fin, &cols, &rows) )
      {
        if( rows != Height || cols != Width ) {
          fprintf(stderr, "Cannot use mask image '%s'; expected %d*%d, 1-bit image\n",
            mask_fn,
            Width, Height);
        }
        else {
          for(int y=0; y<Height; ++y) {
            for(int x=0; x<Width; ++x)
              if( PBM_BLACK == b[y][x] )
                mask.insert( Pos(y,x) );
          }
        }
        free(b);
      }
      pm_close(fin);
    }

  Generate(conns, Width, Height, 0,0,  Width-1,Height-1, mask);

  Path path;
  if( breadcrumbs )
    Solve(conns, Width, Height, 0,0, Width-1,Height-1, path);

  Draw(conns, path, mask, Width,Height, 0,0, Width-1,Height-1);

  if( !mask_fn )
    mask_fn = "";

  printf("\tReproduce: %s %d %d %d %d %s ; "
         "or, %s breadcrumbs: %s %d %d %d %d %s\n",
    argv[0], Width, Height, breadcrumbs, seed, mask_fn,
    (breadcrumbs ? "without" : "with"),
    argv[0], Width, Height, !breadcrumbs, seed, mask_fn);

  return 0;
}


