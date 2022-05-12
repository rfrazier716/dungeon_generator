package main

import (
	"math"
	"math/rand"
	"os"
	"text/template"
	"time"
)

type coord2D struct{
	X float64
	Y float64
}

type coordinateSpace struct{
	xMin float64
	xMax float64
	yMin float64
	yMax float64
}

type svgLine struct{
	X0 float64
	Y0 float64
	X1 float64
	Y1 float64
}

// type alias for a slice of points
type pointCloud = []coord2D

func NewCoord2D(x float64, y float64) coord2D{
	return coord2D{x,y}
}

func NewPointCloud(space coordinateSpace, nPts int) (result pointCloud){
	// generate a new point cloud
	result = make([]coord2D, nPts)
	for i:=0;i<nPts;i++ {
		result[i] = randomPoint(space)
	}
	return
}

func NewCoordinateSpace(xMin float64, xMax float64, yMin float64, yMax float64) coordinateSpace{
	return coordinateSpace{xMin, xMax, yMin, yMax}
}

func randomPoint(space coordinateSpace) coord2D {
	// seed the random number generator
	r1 := rand.New(rand.NewSource(time.Now().UnixNano()))
	x:= r1.Float64()*(space.xMax-space.xMin)+space.xMin
	y:= r1.Float64()*(space.yMax-space.yMin)+space.yMin
	return NewCoord2D(x,y)
}

func distance(p1 coord2D, p2 coord2D) float64 {
	return math.Sqrt(math.Pow(p2.X-p1.X, 2) + math.Pow(p2.Y-p1.Y, 2))
}

func navigate(cloud pointCloud) [][]int{
	//navigates the pointcloud using Prim's Algorithm to find a graph connected all points
	//returns a list of index pairs
	var exists = struct{}{} // short hand for filling the set

	// create a set of unvisited nodes
	unvisitedNodes := make(map[int]struct{}) //golang doesn't have sets... weird
	visitedNodes := make(map[int]struct{}) // set of the nodes we've visited

	// create an adjacency matrix tracking the distance between each point
	adjacencyMatrix := make([][]float64, len(cloud))
	for i:=0;i<len(cloud);i++{
		unvisitedNodes[i]= exists //fill our unvited node sets while we're at it
		adjacencyMatrix[i] = make([]float64, len(cloud)) //initialize the inner slice 
		for j:=0;j<len(cloud);j++{
			adjacencyMatrix[i][j]=distance(cloud[i], cloud[j])
		}
	}

	// we have to start somewhere so add node 0 to the first node visited
	visitedNodes[0] = exists
	delete(unvisitedNodes,0)

	// Create the Path variable which will be returned at the end
	path := make([][]int, len(cloud)-1)


	//now to navigate! we should be able to do this in n-1 iterations where n is the number of points
	for i:=0;i<len(cloud)-1;i++{
		//fmt.Println(visitedNodes,unvisitedNodes)
		path[i]=make([]int, 2) //make a pair to track the minimum
		minDistance:=math.MaxFloat64 // relaly big number for the minimum distance
		//iterate over all nodes currently in the path
		for idx, _ := range visitedNodes{
			// iterate over all adjacent edges to the node to find the minimum distance
			for j, _ := range unvisitedNodes{
				// have to watch out for self loops
				// if we found a new shortest path to a new node update the current path
				if adjacencyMatrix[idx][j]<minDistance{
					//fmt.Printf("Found new minimum distance between nodes %v and %v\n",idx, j)
					path[i][0] = idx
					path[i][1] = j
					minDistance = adjacencyMatrix[idx][j]
				}
			}
		}
		// we've found a path so update the visited and unvisted nodes
		newNode:=path[i][1] // this is the node we've added
		delete(unvisitedNodes, newNode)
		visitedNodes[newNode] = exists
		//fmt.Println(path)

	}
	return path
}

// dumb way to quickly make templates work
func pathToLines(cloud pointCloud, path [][]int) (ret []svgLine) {
	ret = make([]svgLine, len(path))
	for i, indices := range path{
		x0 := cloud[indices[0]].X
		y0 := cloud[indices[0]].Y
		x1 := cloud[indices[1]].X
		y1 := cloud[indices[1]].Y
		ret[i] = svgLine{x0,y0,x1,y1}

	}
	return
}

func main(){
	space := NewCoordinateSpace(-500,500,-500,500)
	cloud := NewPointCloud(space, 100)
	path := navigate(cloud)
	lines := pathToLines(cloud, path)

	// write the point cloud to our template
	tmpl := template.Must(template.ParseFiles("template.tmpl"))
	// create a writer to dump our constellation to
	f, _ := os.Create("out.svg")
	defer f.Close() // defer closering so we don't keep it open
	tmpl.Execute(f, struct{
		Points []coord2D
		Lines []svgLine}{cloud,
		lines})
}