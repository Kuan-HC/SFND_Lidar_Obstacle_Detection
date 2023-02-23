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
		if(left != nullptr)
			delete left;
		if(right != nullptr)
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
		if(root != nullptr)
			delete root;
	}

	void insertHelper(Node *&node, uint32_t depth, const std::vector<float>& point, const int& id, const int& kDim)
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

	void insert(const std::vector<float>& point, const int& id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(root, 0, point, id, point.size());
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>& target, const float& distTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distTol, ids, target.size());

		return ids;
	}

	void searchHelper(const std::vector<float> &target, Node *node, int depth, const float &distTol, std::vector<int> &ids, const int& kDim)
	{
		// 以target 為正方形中心，2倍distTol為正方形邊長，確認node是否在該正方形中
		if (node != nullptr)
		{
			if ((node->point[0] >= target[0] - distTol) && (node->point[0] <= target[0] + distTol) && (node->point[1] >= target[1] - distTol) && (node->point[1] <= target[1] + distTol))
			{
				float dist = sqrt(pow((node->point[0] - target[0]), 2) + pow((node->point[1] - target[1]), 2));
				if (dist <= distTol)
					ids.push_back(node->id);
			}

			// check left or right branch
			uint8_t idx = depth % kDim;
			if (target[idx] - distTol < node->point[idx])
				searchHelper(target, node->left, depth + 1, distTol, ids, kDim);
			if (target[idx] + distTol > node->point[idx])
				searchHelper(target, node->right, depth + 1, distTol, ids, kDim);
		}
	}
};
