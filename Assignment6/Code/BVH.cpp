#include <algorithm>
#include <cassert>
#include <array>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;
    root = recursiveBuild(primitives);
    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);
    
    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        if (splitMethod == SplitMethod::NAIVE) 
        {
            return NaiveBuild(objects);
        }
        else if (splitMethod == SplitMethod::SAH)
        {
            return SahBuild(objects);
        }
    }

    return node;   
}

BVHBuildNode* BVHAccel::NaiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
        centroidBounds =
            Union(centroidBounds, objects[i]->getBounds().Centroid());
    int dim = centroidBounds.maxExtent();
    switch (dim) {
    case 0:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            return f1->getBounds().Centroid().x <
                    f2->getBounds().Centroid().x;
        });
        break;
    case 1:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            return f1->getBounds().Centroid().y <
                    f2->getBounds().Centroid().y;
        });
        break;
    case 2:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
            return f1->getBounds().Centroid().z <
                    f2->getBounds().Centroid().z;
        });
        break;
    }

    auto beginning = objects.begin();
    auto middling = objects.begin() + (objects.size() / 2);
    auto ending = objects.end();

    auto leftshapes = std::vector<Object*>(beginning, middling);
    auto rightshapes = std::vector<Object*>(middling, ending);

    assert(objects.size() == (leftshapes.size() + rightshapes.size()));

    node->left = recursiveBuild(leftshapes);
    node->right = recursiveBuild(rightshapes);

    node->bounds = Union(node->left->bounds, node->right->bounds);
    return node;
}

BVHBuildNode* BVHAccel::SahBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    const int bucketsCount = 20;
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
    {
        centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
    }
    int dim = centroidBounds.maxExtent();
    float bucketWidth = centroidBounds.Diagonal()[dim] / bucketsCount;
    BVHSAHBucket buckets[bucketsCount];

    for (Object* object : objects) {
        int b = bucketsCount * centroidBounds.Offset(object->getBounds().Centroid())[dim];
        b = std::min(b, bucketsCount - 1);
        buckets[b].primCount++;
        buckets[b].bounds = Union(buckets[b].bounds, object->getBounds());
    }

    float minCost = std::numeric_limits<float>::max();
    int minCostSplitBucket = 0;
    for (int i = 0; i < bucketsCount - 1; i++)
    {
        Bounds3 boundsL, boundsR;
        int countL = 0, countR = 0;
        for (int j = 0; j <= i; j++)
        {
            boundsL = Union(boundsL, buckets[j].bounds);
            countL += buckets[j].primCount;
        }
        for (int j = i + 1; j < bucketsCount; j++)
        {
            boundsR = Union(boundsR, buckets[j].bounds);
            countR += buckets[j].primCount;
        }
        float cost = countL * boundsL.SurfaceArea() + countR * boundsR.SurfaceArea();
        if (cost < minCost)
        {
            minCost = cost;
            minCostSplitBucket = i;
        }
    }

    auto mid = std::partition(objects.begin(), objects.end(), [=](auto object) {
        int b = bucketsCount * centroidBounds.Offset(object->getBounds().Centroid())[dim];
        b = std::min(b, bucketsCount - 1);
        return b <= minCostSplitBucket;
    });

    auto leftShapes = std::vector<Object*>(objects.begin(), mid);
    auto rightShapes = std::vector<Object*>(mid, objects.end());

    assert(objects.size() == (leftShapes.size() + rightShapes.size()));

    node->left = recursiveBuild(leftShapes);
    node->right = recursiveBuild(rightShapes);

    node->bounds = Union(node->left->bounds, node->right->bounds);

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{   
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    Intersection inter;
    // TODO Traverse the BVH to find intersection
    std::array<int, 3> dirIsNeg = {int(ray.direction.x > 0), int(ray.direction.y > 0), int(ray.direction.z > 0)};
    if (node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        if (node->object != nullptr) 
        {
            return node->object->getIntersection(ray);
        }
        Intersection hit1 = getIntersection(node->left, ray);
        Intersection hit2 = getIntersection(node->right, ray);
        return hit1.distance < hit2.distance ? hit1 : hit2;
    }
    
    return inter;
}