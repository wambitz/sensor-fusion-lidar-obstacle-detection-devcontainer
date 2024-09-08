#include "render.h"
#include <boost/make_shared.hpp>

// Structure to represent node of kd tree
struct Node
{
    Node(std::vector<float> arr, int setId)
        : point(arr)
        , id(setId)
        , left(nullptr)
        , right(nullptr)
    {
    }

    std::vector<float> point;
    int id;
    boost::shared_ptr<Node> left;
    boost::shared_ptr<Node> right;
};

struct KdTree
{
    KdTree()
        : root(nullptr)
    {
    }

    void insert(std::vector<float> point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        insertHelper(root, 0, point, id);
    }

    void insertHelper(boost::shared_ptr<Node>& node, unsigned int depth, std::vector<float> point, int id)
    {
        // Tree is empty
        if (!node)
        {
            node = boost::make_shared<Node>(point, id);
        }
        else
        {
            // Calculate current dimension
            unsigned int current_dimension = depth % 2;

            if (point[current_dimension] < node->point[current_dimension])
            {
                insertHelper(node->left, depth + 1, point, id);
            }
            else
            {
                insertHelper(node->right, depth + 1, std::move(point), id);
            }
        }
    }

    void searchHelper(std::vector<float> target,
                      boost::shared_ptr<Node> node,
                      int depth,
                      float distanceTolerance,
                      std::vector<int>& ids)
    {
        if (node != nullptr)
        {
            if (node->point[0] >= (target[0] - distanceTolerance) &&
                node->point[0] <= (target[0] + distanceTolerance) &&
                node->point[1] >= (target[1] - distanceTolerance) && node->point[1] <= (target[1] + distanceTolerance))
            {
                float distance = std::sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
                                           (node->point[1] - target[1]) * (node->point[1] - target[1]));
                if (distance <= distanceTolerance)
                {
                    ids.push_back(node->id);
                }
            }

            if ((target[depth % 2] - distanceTolerance) < node->point[depth % 2])
            {
                searchHelper(target, node->left, depth + 1, distanceTolerance, ids);
            }
            if ((target[depth % 2] + distanceTolerance) > node->point[depth % 2])
            {
                searchHelper(target, node->right, depth + 1, distanceTolerance, ids);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTolerance)
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTolerance, ids);
        return ids;
    }

    boost::shared_ptr<Node> root;
};
