#include <iostream>
#include <cstdio>
#include <cmath>
#include <set>
#include <vector>
using namespace std;

#define MAP_SIZE    24
#define RAND rand() / double(RAND_MAX)

typedef struct _Vector {
  double u;
  double v;
} Vector;

const double start_point[2] = {-1.4375, -1.4375};
const double goal_point[2]  = { 1.4375,  1.4375};
const double step = 0.0625;
const double prob = 0.9;
const double radius = 0.0625;
const double check[8][2] = {{ 0.0625,           0},
                            {-0.0625,           0},
                            {      0,      0.0625},
                            {      0,     -0.0625},
                            {   0.0442,    0.0442},
                            {   0.0442,   -0.0442},
                            {  -0.0442,    0.0442},
                            {  -0.0442,   -0.0442}};
const int K = 1000;

vector<Vector> target;

char map[MAP_SIZE][MAP_SIZE] =
{
	{'0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', 'E'},
	{'0', '0', '0', '1', '1', '0', '1', '1', '0', '0', '0', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '0', '0', '0'},
	{'0', '0', '0', '1', '1', '0', '0', '0', '1', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '1', '0', '0', '1', '1'},
	{'0', '0', '1', '1', '0', '0', '0', '0', '1', '1', '1', '0', '0', '1', '0', '0', '0', '0', '1', '1', '0', '0', '1', '1'},
	{'0', '0', '1', '1', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'},
	{'0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'},
	{'1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'},
	{'1', '1', '0', '0', '1', '0', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0'},
	{'0', '0', '1', '1', '1', '0', '0', '0', '0', '1', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '1', '1', '0'},
	{'0', '0', '1', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '1', '1', '0'},
	{'0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '1', '0'},
	{'0', '0', '0', '0', '0', '1', '1', '1', '0', '1', '1', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0'},
	{'1', '1', '0', '0', '0', '1', '1', '1', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'},
	{'1', '1', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '1', '0', '0', '0', '0', '1', '0', '0', '1', '0', '0'},
	{'1', '1', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '1', '0', '0', '1', '1', '1', '0', '0', '1', '0', '0'},
	{'0', '0', '1', '1', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '0', '0', '0', '0', '1', '0', '0'},
	{'0', '0', '1', '1', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'},
	{'0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '1', '0', '0', '0', '1', '1', '0', '0', '0', '0'},
	{'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '0', '0', '1', '1', '0', '0', '0', '0'},
	{'0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '0', '1', '1', '1', '1', '0', '0', '1', '1', '0', '0', '0', '0'},
	{'0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '0', '1', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'},
	{'0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0'},
	{'0', '0', '1', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0'},
	{'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1', '1', '0', '0'},
};

class node
{
public:
    double x;
    double y;
    double distance;

    node(double _x, double _y)
    {
        x = _x;
        y = _y;
        distance = sqrt(pow((x - goal_point[0]), 2) + pow((y - goal_point[1]), 2));
    }

    bool operator <(const node& n) const
    {
        return distance < n.distance;
    }

    bool threshold()
    {
        return (x > 1.375) && (x < 1.5) && (y > 1.375) &&( y < 1.5);
    }

    bool collision()
    {
        for(int i = 0;i < 8;i++)
        {
            //cout << "i = " << i << endl;
            double sample_x = x + check[i][0];
            double sample_y = y + check[i][1];

            // shift position
            sample_x = sample_x + 1.5;
            sample_y = sample_y + 1.5;
            //cout << sample_x << " " << sample_y << endl;
            if(sample_x < 0 || sample_y < 0 || sample_x > 3 || sample_y > 3)
                return true;

            // which block
            int dx = 23 - int(sample_x / 0.125);
            int dy = sample_y / 0.125;

            //cout << "dx = " << dx  << " " << "dy = " << dy << endl;

            if(map[dx][dy] == '1')
                return true;
        }
        return false;
    }
};

set<node> tree;
set<node>::iterator tree_it;

void display_tree()
{
    set<node>::iterator iter = tree.begin();
    for(;iter != tree.end();iter++)
        cout << iter->x << " " << iter->y << endl;
}

node extend(node qnearest, node qrand)
{
    double extend_x = 0.0, extend_y = 0.0;
    double dis = sqrt(pow((qrand.x - qnearest.x), 2) + pow((qrand.y - qnearest.y), 2));

    extend_x = (qrand.x - qnearest.x) / dis * step;
    extend_y = (qrand.y - qnearest.y) / dis * step;

    double xx = qnearest.x + extend_x;
    double yy = qnearest.y + extend_y;
    //cout << xx << " " << yy << endl;

    node qnew(xx, yy);

    return qnew;
}

node sample()
{
    double p = RAND;
    //cout << p << endl;
    double new_x = 0.0, new_y = 0.0;
    if(p > 0 && p < prob)
    {
        new_x = RAND*3 - 1.5;
        new_y = RAND*3 - 1.5;
    }else
    {
        new_x = goal_point[0];
        new_y = goal_point[1];
    }
    //cout << new_x << " " << new_y << endl;
    node n(new_x, new_y);
    return n;
}

bool buildRRT()
{
    // init RRT
    node n(start_point[0], start_point[1]);
    Vector v = {-1.4375, -1.4375};
    target.push_back(v);
    tree.insert(n);
    tree_it = tree.begin();

    for(int k = 0;k < K;k++)
    {
        //cout << "k = " << k << endl;
        node qrand = sample();
        tree_it = tree.begin();
        node qnearest = *tree_it;
        if((qnearest.x - target[target.size()-1].u) > 1e-8 || (qnearest.y - target[target.size()-1].v) > 1e-8)
        {
            Vector v = {qnearest.x, qnearest.y};
            //cout << qnearest.x << " " << qnearest.y << endl;
            target.push_back(v);
        }    
        
        //cout << qnearest.x << " " << qnearest.y << endl;

        if(qnearest.threshold())
            return true;
        //cout << "here" << endl;
        node qnew = extend(qnearest, qrand);
        //cout << qnew.x << " " << qnew.y << endl;
        //cout << "here" << endl;

        if(!qnew.collision())
        {
            //cout << qnew.x << " " << qnew.y << endl;
            tree.insert(qnew);
        }

        //cout << "here" << endl;

        //display_tree();
    }

    return false;
}

int main()
{
    //srand(time(NULL));
    buildRRT();
    cout << target.size() << endl;
    for(int i = 0; i < target.size();i++)
    {
        printf("{%f, %f},\n", target[i].u, target[i].v);
    }
    return 0;
}
