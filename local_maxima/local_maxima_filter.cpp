#include "local_maxima_filter.h"

namespace kamaz {
    namespace hagen{
        
    LocalMaximaFilter::LocalMaximaFilter(){
        uf = new UnionFindDS();
    }

    LocalMaximaFilter::~LocalMaximaFilter(){
        //delete uf;
    }
    
    float LocalMaximaFilter::get_pixel_value(const cv::Mat& img, std::tuple<int, int, float> p){
        cv::Scalar intensity = img.at<float>(std::get<0>(p), std::get<1>(p));                        
        return intensity[0];
    }

    void LocalMaximaFilter::iter_neighbors(std::tuple<int, int, float> p, int w, int h, std::vector<std::tuple<int, int, float>>& result, const cv::Mat& img){
        int y = std::get<0>(p);
        int x = std::get<1>(p);
        std::vector<std::tuple<int, int>> neigh;
        int  filter[3] = {-1, 0, 1};
        for(const int &i : filter){
            for(const int &j : filter){
                std::tuple<int, int> index(y+j, x+i);
                neigh.push_back(index);
            }
        }
        for(const std::tuple<int, int>& item_ : neigh){
            int j = std::get<0>(item_);
            int i = std::get<1>(item_);
            if (j < 0 or j >= h){
                 continue;
            }
            if (i < 0 or i >= w){
                continue;
            }
            if (j == y and i == x){
                continue;
            }
            cv::Scalar intensity = img.at<float>(j, i);                         
            std::tuple<int, int, float> selected_item(j, i, intensity[0]);
            result.push_back(selected_item);
        }
        return;
    } 

    void LocalMaximaFilter::persistence(const cv::Mat& img, cv::Mat& filtered_image){
        int h = img.rows;
        int w = img.cols;

        std::vector<std::tuple<int, int, float>> indices;
        for(int i=0; i<h; i++){
            for(int j=0; j<w; j++){
                cv::Scalar intensity = img.at<float>(i, j);
                std::tuple<int, int, float> index_(i, j, intensity[0]);
                indices.push_back(index_);
            }
        }

        // // Get indices orderd by value from high to low
        __gnu_parallel::sort(indices.rbegin(), indices.rend(),
         [&](const std::tuple<int, int, float> a, const std::tuple<int, int, float> b){
             return std::get<2>(a) < std::get<2>(b); 
         });


        std::map<std::tuple<int, int, float>, groups> group0;
        // Process pixels from high to low
        int k = 0;
        for (const auto& p : indices){
            float v = std::get<2>(p);
            std::vector<std::tuple<int, int, float>> ni_old;
            std::vector<std::tuple<float, std::tuple<int, int, float>>> nc;
            std::vector<std::tuple<int, int, float>> item_list;
            iter_neighbors(p, w, h, item_list, img);
            for(const std::tuple<int, int, float>& _object : item_list){
                if (uf->is_contains(_object)){
                        auto iy = uf->get_items(_object);
                        ni_old.push_back(iy);
                }
            }

            std::set<std::tuple<int, int, float>> ni(ni_old.begin(), ni_old.end());
            int couuu=0;
            for(auto ancesttor : ni){
                couuu++;
                std::tuple<float, std::tuple<int, int, float>> 
                    __item(std::get<2>(ancesttor), ancesttor);
                nc.push_back(__item); 
            }

            __gnu_parallel::sort(nc.rbegin(), nc.rend(),
                [](const std::tuple<float, std::tuple<int, int, float>> a, const std::tuple<float, std::tuple<int, int, float>> b){
                    return std::get<0>(a) > std::get<0>(b); 
                });

            if(k == 0){
                groups gr;
                gr.p1 = v;
                gr.p2 = v;
                gr.p3 = std::make_tuple(0, 0, 0.0);
                group0[p]= gr;
            }

            uf->add(p, -k);
            if(nc.size() > 0){
                std::vector<std::tuple<int, int, float>> indexes;
                indexes.push_back(std::get<1>(nc[0]));
                indexes.push_back(p);
                uf->union_f(indexes);
                for(std::size_t l=1; l<nc.size(); ++l){
                    auto bl = std::get<0>(nc[l]);
                    auto q = std::get<1>(nc[l]);
                    if (uf->is_contains(q)){
                            auto corresponding_value = uf->get_items(q);
                            if(group0.find(corresponding_value) == group0.end()){
                                groups gr;
                                gr.p1 = bl;
                                gr.p2 = bl-v;
                                gr.p3 = p;
                                group0[corresponding_value] = gr; 
                                std::vector<std::tuple<int, int, float>> _item_list;
                                iter_neighbors(corresponding_value, w, h, _item_list, img);
                                for(const std::tuple<int, int, float>& _object : item_list){
                                    applyBilateralFilter(img, filtered_image, std::get<0>(_object), std::get<1>(_object), 3, h, w, 1.2, 1.2);
                                }
                                iter_neighbors(p, w, h, _item_list, img);
                                for(const std::tuple<int, int, float>& _object : item_list){
                                    applyBilateralFilter(img, filtered_image, std::get<0>(_object), std::get<1>(_object), 3, h, w, 1.2, 1.2);
                                }
                            }
                            indexes[1] = corresponding_value;
                            uf->union_f(indexes);
                        }
                }
            }
            k+=1;
        }
        return;
    }

    void LocalMaximaFilter::persistence_and_save_data(const cv::Mat& img, cv::Mat& filtered_image, int indexi){
        int h = img.rows;
        int w = img.cols;
        
        auto cmp = [](std::tuple<int, int, float> l, std::tuple<int, int, float> r) 
        { return std::get<2>(l) < std::get<2>(r);};
    
        std::priority_queue<std::tuple<int, int, float>
        , std::vector<std::tuple<int, int, float>>, decltype(cmp)> indices_queue(cmp);

        std::vector<std::tuple<int, int, float>> indices;
        for(int i=0; i<h; i++){
            for(int j=0; j<w; j++){
                cv::Scalar intensity = img.at<float>(i, j);
                std::tuple<int, int, float> index_(i, j, intensity[0]);
                indices_queue.push(index_);
                // indices.push_back(index_);
            }
        }

        while(!indices_queue.empty()) {
            indices.push_back(indices_queue.top());
            indices_queue.pop();
        }
        // // // Get indices orderd by value from high to low
        // __gnu_parallel::sort(indices.rbegin(), indices.rend(),
        //  [&](const std::tuple<int, int, float> a, const std::tuple<int, int, float> b){
        //      return std::get<2>(a) < std::get<2>(b); 
        //  });


        std::map<std::tuple<int, int, float>, groups> group0;
        // Process pixels from high to low
        int k = 0;
        for (const auto& p : indices){
            float v = std::get<2>(p);
            std::vector<std::tuple<int, int, float>> ni_old;
            std::vector<std::tuple<float, std::tuple<int, int, float>>> nc;
            std::vector<std::tuple<int, int, float>> item_list;
            iter_neighbors(p, w, h, item_list, img);
            for(const std::tuple<int, int, float>& _object : item_list){
                if (uf->is_contains(_object)){
                        auto iy = uf->get_items(_object);
                        ni_old.push_back(iy);
                }
            }

            std::set<std::tuple<int, int, float>> ni(ni_old.begin(), ni_old.end());
            int couuu=0;
            
            auto cmp_ni = [](std::tuple<float, std::tuple<int, int, float>> l, std::tuple<float, std::tuple<int, int, float>> r) 
            { return std::get<0>(l) > std::get<0>(r);};

            std::priority_queue<std::tuple<float, std::tuple<int, int, float>>
            , std::vector<std::tuple<float, std::tuple<int, int, float>>>, decltype(cmp_ni)> ni_queue(cmp_ni);

            for(auto ancesttor : ni){
                couuu++;
                std::tuple<float, std::tuple<int, int, float>> 
                    __item(std::get<2>(ancesttor), ancesttor);
                ni_queue.push(__item); 
            }
            
            while(!ni_queue.empty()) {
                nc.push_back(ni_queue.top());
                ni_queue.pop();
            }

            // __gnu_parallel::sort(nc.rbegin(), nc.rend(),
            //     [](const std::tuple<float, std::tuple<int, int, float>> a, const std::tuple<float, std::tuple<int, int, float>> b){
            //         return std::get<0>(a) > std::get<0>(b); 
            //     });

            if(k == 0){
                groups gr;
                gr.p1 = v;
                gr.p2 = v;
                gr.p3 = std::make_tuple(0, 0, 0.0);
                group0[p]= gr;
            }

            uf->add(p, -k);
            if(nc.size() > 0){
                std::vector<std::tuple<int, int, float>> indexes;
                indexes.push_back(std::get<1>(nc[0]));
                indexes.push_back(p);
                uf->union_f(indexes);
                for(std::size_t l=1; l<nc.size(); ++l){
                    auto bl = std::get<0>(nc[l]);
                    auto q = std::get<1>(nc[l]);
                    if (uf->is_contains(q)){
                            auto corresponding_value = uf->get_items(q);
                            if(group0.find(corresponding_value) == group0.end()){
                                groups gr;
                                gr.p1 = bl;
                                gr.p2 = bl-v;
                                gr.p3 = p;
                                group0[corresponding_value] = gr; 
                                std::vector<std::tuple<int, int, float>> _item_list;
                                iter_neighbors(corresponding_value, w, h, _item_list, img);
                                for(const std::tuple<int, int, float>& _object : item_list){
                                    applyBilateralFilter(img, filtered_image, std::get<0>(_object), std::get<1>(_object), 3, h, w, 1.2, 1.2);
                                }
                                iter_neighbors(p, w, h, _item_list, img);
                                for(const std::tuple<int, int, float>& _object : item_list){
                                    applyBilateralFilter(img, filtered_image, std::get<0>(_object), std::get<1>(_object), 3, h, w, 1.2, 1.2);
                                }
                            }
                            indexes[1] = corresponding_value;
                            uf->union_f(indexes);
                        }
                }
            }
            k+=1;
        }

        std::vector<std::tuple<int, int, float, float, float, int, int, float>> groupn;

        for (auto const k : group0){
            auto key = k.first;
            auto val = k.second;
            auto val_k = val.p3;
            std::tuple<int, int, float, float, float, int,
             int, float> m(std::get<0>(key), std::get<1>(key)
             , std::get<2>(key), val.p1, val.p2
             , std::get<0>(val_k), std::get<1>(val_k), std::get<2>(val_k));
            groupn.push_back(m);
        }
        std::sort(groupn.rbegin(), groupn.rend(),
                [](const std::tuple<int, int, float, float, float, int, int, float> a
                , const std::tuple<int, int, float, float, float, int, int, float> b){
                    return std::get<2>(a) > std::get<2>(b); 
                });
        
        std::vector<float> np_mat;
        int groupn_size = groupn.size();
        for(auto const vals : groupn){
            np_mat.push_back((float)std::get<0>(vals));  
            np_mat.push_back((float)std::get<1>(vals));  
            np_mat.push_back((float)std::get<2>(vals));  
            np_mat.push_back((float)std::get<3>(vals));
            np_mat.push_back((float)std::get<4>(vals));  
            np_mat.push_back((float)std::get<5>(vals));  
            np_mat.push_back((float)std::get<6>(vals));  
            np_mat.push_back((float)std::get<7>(vals));   
        }
        
        std::string location = "/dataset/result/local_maxima_" + std::to_string(indexi) + ".npy";
        cnpy::npy_save(location ,&np_mat[0],{1, groupn_size, 8},"w");
        return;
    }



    void LocalMaximaFilter::print_tuple(std::tuple<int, int> object){
        std::cout<< "( " << std::get<0>(object) << " , " << std::get<1>(object) << ")" << std::endl;
    }

    float LocalMaximaFilter::distance(int x, int y, int i, int j) {
        return float(sqrt(pow(x - i, 2) + pow(y - j, 2)));
    }

    float LocalMaximaFilter::gaussian(float x, float sigma) {
        return exp(-(pow(x, 2))/(2 * pow(sigma, 2))) / (2 * CV_PI * pow(sigma, 2));
    }

    void LocalMaximaFilter::applyBilateralFilter(const cv::Mat& source, cv::Mat& filteredImage, int x_index, int y_index, int diameter, int h, int w, float sigmaI, float sigmaS) {
        float iFiltered = 0;
        float wP = 0;
        int neighbor_x = 0;
        int neighbor_y = 0;
        int half = diameter / 2;
        for(int i = 0; i < diameter; i++) {
            for(int j = 0; j < diameter; j++) {
                neighbor_x = x_index - (half - i);
                neighbor_y = y_index - (half - j);
                if (neighbor_y < 0 or neighbor_y >= w){
                continue;
                }
                if (neighbor_x < 0 or neighbor_x >= h){
                    continue;
                }
                float gi = gaussian(source.at<float>(neighbor_x, neighbor_y) - source.at<float>(x_index, y_index), sigmaI);
                //float gs = gaussian(distance(x_index, y_index, neighbor_x, neighbor_y), sigmaS);
                //float w_p = gi * gs;
                float w_p = gi;
                iFiltered = iFiltered + source.at<float>(neighbor_x, neighbor_y) * w_p;
                wP = wP + w_p;
            }
        }
        if(wP >0){
            iFiltered = iFiltered / wP;
            filteredImage.at<float>(x_index, y_index) = iFiltered;
        }
    }
}
}



