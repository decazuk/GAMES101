//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject) const
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        // Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection interLight;
    float pdf;
    sampleLight(interLight, pdf);
    Intersection interObj = intersect(ray);
    if (interObj.happened) 
    {
        if (interObj.m->hasEmission() && depth == 0) { //solve light is black
            return interObj.m->getEmission();
        }
        Vector3f x = interLight.coords;
        Vector3f ws = interLight.coords - interObj.coords;
        float lightDistance = ws.norm();
        ws = normalize(ws);
        Vector3f wo = ray.direction;
        Vector3f NN = interLight.normal;
        float R = (interLight.coords - interObj.coords).norm();
        const Ray shotRay = Ray(interObj.coords, ws);
        Intersection interShotRay = intersect(shotRay);
        Vector3f L_dir;
        Vector3f L_indir;
        if (interShotRay.distance > lightDistance - EPSILON)
        {
            L_dir = interLight.emit * interObj.m->eval(wo, ws, interObj.normal) * dotProduct(ws, interObj.normal) * dotProduct(-ws, interLight.normal) / (R * R) / pdf;
        }
        
        if (get_random_float() < RussianRoulette)
        {
            Vector3f wi = normalize(interObj.m->sample(wo, interObj.normal));
            Ray wiRay = Ray(interObj.coords, wi);
            Intersection interWiRay = intersect(wiRay);
            if (interWiRay.happened && !interWiRay.m->hasEmission()) {
                float pdf_hemi = interObj.m->pdf(wo, wi, interObj.normal);
                if (pdf_hemi > EPSILON) { // solve white noise
                    Vector3f casWiRay = castRay(wiRay, depth + 1);
                    L_indir = casWiRay * interObj.m->eval(wo, wi, interObj.normal) * dotProduct(wi, interObj.normal) / pdf_hemi / RussianRoulette;
                }
            }
        }
        return L_dir + L_indir;
    }
    else
    {
        return Vector3f();
    }
}