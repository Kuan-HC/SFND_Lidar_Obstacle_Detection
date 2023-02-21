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
		delete left;
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
		delete root;
	}

	void insertHelper(Node *&node, uint32_t depth, std::vector<float> point, int id)
	{
		if (node == nullptr)
			node = new Node(point, id);
		else
		{
			uint8_t idx = depth & 1;

			if (point[idx] < (node->point[idx]))
				insertHelper(node->left, depth + 1, point, id);
			else
				insertHelper(node->right, depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}

	void searchHelper(const std::vector<float> &target, Node *node, int depth, const float &distanceTol, std::vector<int> &ids)
	{
		// 以target 為正方形中心，2倍distanceTol為正方形邊長，確認node是否在該正方形中
		if (node != nullptr)
		{
			if ((node->point[0] >= target[0] - distanceTol) && (node->point[0] <= target[0] + distanceTol) && (node->point[1] >= target[1] - distanceTol) && (node->point[1] <= target[1] + distanceTol))
			{
				float dist = sqrt(pow((node->point[0] - target[0]), 2) + pow((node->point[1] - target[1]), 2));
				if (dist <= distanceTol)
					ids.push_back(node->id);
			}

			// 之後該往左還是右分支走
			uint8_t idx = depth & 1;
			if (target[idx] - distanceTol < node->point[idx])
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			if (target[idx] + distanceTol > node->point[idx])
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}
	}
};
