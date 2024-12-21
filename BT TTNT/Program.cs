using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BT_TTNT
{
    internal class Program
    {
        static Random random = new Random();
        //Tạo cây
        static TreeNode GenerateTree(int nodeCount, out List<TreeNode> nodes)
        {
            nodes = new List<TreeNode>();
            for (int i = 0; i < nodeCount; i++)
            {
                int heuristic = random.Next(1, 50);//heuristic từ 1 đến 50
                nodes.Add(new TreeNode($"Node{i + 1}", heuristic));
            }
            foreach (var node in nodes)
            {
                int edgeCount = random.Next(1, 4); //Số nhánh con
                for (int i = 0; i < edgeCount; i++)
                {
                    var targetNode = nodes[random.Next(nodes.Count)];
                    if (targetNode != node)
                    {
                        int cost = random.Next(1, 21); // Cost từ 1 đến 20
                        node.Edges.Add(new Edge(targetNode, cost));
                    }
                }
            }
            return nodes[0]; // trả về node gốc
        }
        static void PrintTree(TreeNode node, HashSet<TreeNode> visited, string prefix = "")
        {
            if (visited.Contains(node))
                return;

            visited.Add(node);

            Console.WriteLine($"{prefix}{node.Name} (Heuristic: {node.Heuristic})");
            foreach (var edge in node.Edges)
            {
                Console.WriteLine($"{prefix}  -> {edge.TargetNode.Name} (Cost: {edge.Cost})");
                PrintTree(edge.TargetNode, visited, prefix + "    ");
            }
        }
        static void Main(string[] args)
        {
            Console.OutputEncoding = Encoding.UTF8;//Cho phép out ở dạng UTF8 để không lỗi font tiếng việt
            int nodeCount = random.Next(10,20);//Lấy random tổng số node từ 10 đến 20
            List<TreeNode> nodes;
            var root = GenerateTree(nodeCount, out nodes);//Tạo tree
            PrintTree(root, new HashSet<TreeNode>());
            var startNode = root;//Lấy node gốc làm start node
            int indexGoalNode = random.Next(1, nodeCount - 1);//lấy random từ nodes để làm goalnode
            var goalNode = nodes[indexGoalNode];

            int depthlength = 2;//Sử dụng thuật toán dfs với depthlength = 2
            var dfsSubset = DFSAlgorithm.DFSLimit(startNode, depthlength);

            //Từ defSubset bắt đầu sử dụng thuật toán a* để tìm đường đi
            var path = AStarAlgorithm.AStarOnSubset(startNode, goalNode, dfsSubset);
            if (path == null)
            {
                Console.WriteLine("Không tìm thấy đường đi");
                return;
            }
            Console.WriteLine("Tìm được đường đi:");
            foreach (var node in path)
            {
                Console.Write($"(Node: {node.Name}, H: {node.Heuristic}) -> ");
            }
        }
    }
    class AStarAlgorithm
    {
        public static List<TreeNode> FindPath(TreeNode start, TreeNode goal)
        {
            var openSet = new List<TreeNode> { start };
            var cameFrom = new Dictionary<TreeNode, TreeNode>();
            var gScore = new Dictionary<TreeNode, int> { [start] = 0 };
            var fScore = new Dictionary<TreeNode, int> { [start] = start.Heuristic };

            while (openSet.Count > 0)
            {
                // Lấy node có fScore thấp nhất
                var current = openSet.OrderBy(node => fScore.ContainsKey(node) ? fScore[node] : int.MaxValue).First();

                //Nếu current node == goal node -> cấu trúc lại đường đi từ danh sách các điểm đã tới để return
                if (current == goal)
                    return ReconstructPath(cameFrom, current);

                openSet.Remove(current);

                foreach (var edge in current.Edges)
                {
                    var neighbor = edge.TargetNode;
                    var totalScore = gScore[current] + edge.Cost;

                    //Nếu node neightbor chưa duyệt và có totalScore < score của neighbor
                    if (!gScore.ContainsKey(neighbor) || totalScore < gScore[neighbor])
                    {
                        cameFrom[neighbor] = current;
                        gScore[neighbor] = totalScore;
                        fScore[neighbor] = totalScore + neighbor.Heuristic;

                        //thêm neightbor node vào tập duyệt
                        if (!openSet.Contains(neighbor))
                            openSet.Add(neighbor);
                    }
                }
            }

            return null; // Không tìm thấy đường đi
        }
        public static List<TreeNode> AStarOnSubset(TreeNode start, TreeNode goal, List<TreeNode> subset)
        {
            var openSet = new List<TreeNode> { start };
            var cameFrom = new Dictionary<TreeNode, TreeNode>();
            var gScore = new Dictionary<TreeNode, int> { [start] = 0 };
            var fScore = new Dictionary<TreeNode, int> { [start] = start.Heuristic };

            while (openSet.Count > 0)
            {
                // Lấy node có fScore thấp nhất
                var current = openSet.OrderBy(node => fScore.ContainsKey(node) ? fScore[node] : int.MaxValue).First();

                //Nếu current node == goal node -> cấu trúc lại đường đi từ danh sách các điểm đã tới để return
                if (current == goal)
                    return ReconstructPath(cameFrom, current);

                openSet.Remove(current);

                foreach (var edge in current.Edges)
                {
                    var neighbor = edge.TargetNode;
                    if (!subset.Contains(neighbor)) continue; // Chỉ duyệt các node trong tập DFS

                    //Tính chi phí dự kiến
                    var totalScore = gScore[current] + edge.Cost;

                    //Nếu node neightbor chưa duyệt và có totalScore < score của neighbor
                    if (!gScore.ContainsKey(neighbor) || totalScore < gScore[neighbor])
                    {
                        cameFrom[neighbor] = current;
                        gScore[neighbor] = totalScore;
                        fScore[neighbor] = totalScore + neighbor.Heuristic;

                        //thêm neightbor node vào tập duyệt
                        if (!openSet.Contains(neighbor))
                            openSet.Add(neighbor);
                    }
                }
            }

            return null; // Không tìm thấy đường đi
        }
        //Thiết lập danh sách đường đi
        private static List<TreeNode> ReconstructPath(Dictionary<TreeNode, TreeNode> cameFrom, TreeNode current)
        {
            var path = new List<TreeNode> { current };
            while (cameFrom.ContainsKey(current))
            {
                current = cameFrom[current];
                path.Insert(0, current);
            }
            return path;
        }
    }
    class DFSAlgorithm
    {
        public static List<TreeNode> DFSLimit(TreeNode start, int limit)
        {
            var result = new List<TreeNode>();
            var stack = new Stack<(TreeNode, int)>();
            stack.Push((start, 0));

            while (stack.Count > 0)
            {
                var (current, depth) = stack.Pop();
                if (depth > limit) continue;

                result.Add(current);

                foreach (var edge in current.Edges)
                {
                    stack.Push((edge.TargetNode, depth + 1));
                }
            }
            return result;
        }
        public static List<TreeNode> FindPath(TreeNode start, TreeNode goal)
        {
            var stack = new Stack<(TreeNode, List<TreeNode>)>();
            var visited = new HashSet<TreeNode>();

            //Thêm node gốc vào ngăn xếp
            stack.Push((start, new List<TreeNode> { start }));

            while (stack.Count > 0)
            {
                var (current, path) = stack.Pop();

                //nếu node hiện tại == goal -> trả về
                if (current == goal)
                    return path;

                //đánh dấu node đã visit
                visited.Add(current);

                //thêm node con vào stack
                foreach (var edge in current.Edges)
                {
                    var neighbor = edge.TargetNode;

                    if (!visited.Contains(neighbor))
                    {
                        var newPath = new List<TreeNode>(path) { neighbor };
                        stack.Push((neighbor, newPath));
                    }
                }
            }

            return null; // Không tìm thấy đường đi
        }
    }
    class TreeNode
    {
        public string Name { get; set; }
        public int Heuristic { get; set; }
        public List<Edge> Edges { get; set; } = new List<Edge>();
        public TreeNode(string name, int heuristic)
        {
            Name = name;
            Heuristic = heuristic;
        }
    }
    class Edge
    {
        public TreeNode TargetNode { get; set; }
        public int Cost { get; set; }
        public Edge(TreeNode targetNode, int cost)
        {
            TargetNode = targetNode;
            Cost = cost;
        }
    }
}