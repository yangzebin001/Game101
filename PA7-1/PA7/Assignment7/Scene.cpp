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
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
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
    Intersection intersection = Scene::intersect(ray);
    if(intersection.happened) {
        // std::cout<<1<<std::endl;
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Material *m = intersection.m;
        // if(m->hasEmission())
        //     std::cout<<1<<std::endl;

        // init L_dir and L_indir
        Vector3f L_dir(0.0), L_indir(0.0);

        // Uniformly sample the light at x (pdf_light = 1 / A)
        Intersection intersection_light;
        float pdf_light;
        sampleLight(intersection_light, pdf_light);
        // std::cout<<intersection_light.coords.x<<" "<<intersection_light.coords.y<<" "<<intersection_light.coords.z<<std::endl;

        // Shoot a ray from p to x
        Vector3f dir_p_x = (intersection_light.coords - hitPoint).normalized();
        Ray ray_p_x(hitPoint + EPSILON * N, dir_p_x);
        // std::cout<<hitPoint.x<<" "<<hitPoint.y<<" "<<hitPoint.z<<std::endl;
        // std::cout<<dir_p_x.x<<" "<<dir_p_x.y<<" "<<dir_p_x.z<<std::endl;
        Intersection intersection_p_x = Scene::intersect(ray_p_x);
        // if(intersection_p_x.happened)
            // std::cout<<pdf_light<<" "<<intersection_p_x.distance<<std::endl;

        // If the ray is not blocked in the middle
        if(intersection_p_x.happened && intersection_p_x.m->hasEmission()) {
            // std::cout<<1<<std::endl;
            Vector3f NN = intersection_p_x.normal;
            L_dir = intersection_p_x.m->m_emission * m->eval(ray.direction, dir_p_x, N) * dotProduct(dir_p_x, N) * dotProduct(-dir_p_x, NN) / intersection_p_x.distance / pdf_light;
        }

        // Test Russian Roulette with probability RussianRoulette
        if(get_random_float() <= RussianRoulette) {
            // Trace a ray r(p, wi)
            // std::cout<<1<<std::endl;
            Vector3f dir_i = m->sample(ray.direction, N).normalized();
            Ray ray_p_diri(hitPoint, dir_i);
            Intersection intersection_p_diri = Scene::intersect(ray_p_diri);
            
            // If ray r hit a non-emitting object at q
            if(intersection_p_diri.happened && !intersection_p_diri.m->hasEmission()) {
                L_indir = castRay(ray_p_diri, depth+1) * m->eval(ray.direction, dir_i, N) * dotProduct(dir_i, N) / m->pdf(ray.direction, dir_i, N) / RussianRoulette;
            }
        }

        return m->getEmission() + L_dir + L_indir;
    } else {
        return Vector3f(0,0,0);
    }
}
