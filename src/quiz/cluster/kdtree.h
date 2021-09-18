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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	/* This is a helper function that implements a recursive approach to insert a node */
	void insertHelper(Node** node, uint depth, std::vector<float> point, int id){

		// Check if the tree is empty
		if(*node == NULL){
			*node = new Node(point, id);
		}else{
			// Calculate current dimension
			uint cd = depth%2; // it is 2 because is a 2D example

			if (point[cd]<((*node)->point[cd]))
			{
				insertHelper(&((*node)->left), depth+1, point, id);
			}else{
				insertHelper(&((*node)->right), depth+1, point, id);
			}		
		}

	}

	void insert(std::vector<float> point, int id)
	{
		// this function insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




