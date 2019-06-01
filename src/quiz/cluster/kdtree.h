/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;
	KdTree()
	: root(NULL)
	{}

	void insertRec(Node **node, int depth, std::vector<float> point, int id) {
		if (*node == NULL){
			*node = new Node(point, id);
		} else {
			uint cd = depth % 2;
			if (point[cd] > ((*node)->point[cd])) {
				insertRec(&((*node)->right), depth+1, point, id);
			} else if (point[cd] < ((*node)->point[cd])) {
				insertRec(&((*node)->left), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertRec(&root, 1, point, id);
	}

	void searchHelper(std::vector<float> point, Node *node, int depth, float distTol, std::vector<int> &ids) {
		if (node != NULL){
			if ((node->point[0]>=(point[0]-distTol) && node->point[0]<=(point[0]+distTol)) && 
					(node->point[1]>=(point[1]-distTol) && node->point[1]<=(point[1]+distTol)) ) {
						float x_dist = node->point[0]-point[0];
						float y_dist = node->point[1]-point[1];
						float distance_points = sqrt((x_dist*x_dist)+(y_dist*y_dist));
						if (distance_points <= distTol) {
							ids.push_back(node->id);
						}
					}
			if ((point[depth%2]-distTol)<node->point[depth%2])
				searchHelper(point, node->left, depth+1, distTol, ids);
			if ((point[depth%2]+distTol)>node->point[depth%2])
				searchHelper(point, node->right, depth+1, distTol, ids);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




