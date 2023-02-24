/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		if (left != nullptr)
			delete left;
		if (right != nullptr)
			delete right;
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL) {}

	~KdTree()
	{
		if (root != nullptr)
			delete root;
		std::cout << "[+] KDtree delete" << std::endl;
	}

	void insertHelper(Node *&node, uint32_t depth, const std::vector<float> &point, const int &id, const int &kDim)
	{
		if (node == nullptr)
			node = new Node(point, id);
		else
		{
			uint8_t idx = depth % kDim;

			if (point[idx] < (node->point[idx]))
				insertHelper(node->left, depth + 1, point, id, kDim);
			else
				insertHelper(node->right, depth + 1, point, id, kDim);
		}
	}

	void insert(const std::vector<float> &point, const int &id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(root, 0, point, id, point.size());
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float> &target, const float &distTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distTol, ids, target.size());

		return ids;
	}

	void searchHelper(const std::vector<float> &target, Node *node, int depth, const float &distTol, std::vector<int> &ids, const int &kDim)
	{
		// take target as the center and draw a square to check if node is inside of the square
		if (node != nullptr)
		{
			bool inside = true;
			float dist = 0.0f;
			for (int i = 0; i < kDim; ++i)
			{
				dist += pow((node->point[i] - target[i]), 2);
				if ((node->point[i] < target[i] - distTol) || (node->point[i] > target[i] + distTol))
				{
					inside = false;
					break;
				}
			}

			if (inside == true && sqrt(dist) <= distTol)
				ids.push_back(node->id);

			// check left or right branch
			uint8_t idx = depth % kDim;
			if (target[idx] - distTol < node->point[idx])
				searchHelper(target, node->left, depth + 1, distTol, ids, kDim);
			if (target[idx] + distTol > node->point[idx])
				searchHelper(target, node->right, depth + 1, distTol, ids, kDim);
		}
	}
};
