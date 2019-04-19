namespace RRT {

    struct TreeNode
    {
        TreeNode(){}
//        TreeNode(const double x_, const double y_, const int parent_id_):
//                        x_coordinate(x_),y_coordinate(y_),parent_id(parent_id_){}
        ~TreeNode(){}

        double x_coordinate;
        double y_coordinate;
        int parent_id;
        int node_id;
        double cost;
        TreeNode *parent_node;

        bool operator==(const TreeNode &rhs) const
        {
            return (x_coordinate == rhs.x_coordinate) &&
                   (y_coordinate == rhs.y_coordinate);
        }

    };


}
