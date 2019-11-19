#include "local_maxima_filter.h"

namespace kamaz {
    namespace hagen{

    double LocalMaximaFilter::get_pixel_value(Pixel p){
        // cv::Scalar intensity = img.at<double>(p.index/img_width, p.index%img_width);
        cv::Scalar intensity = depth_img_row_vector.at(p.index);
        return intensity[0];
    }

    void LocalMaximaFilter::iter_neighbors(Pixel p, std::vector<Pixel>& result){
        int x = p.index / img_width;
        int y = p.index % img_width;
        std::vector<Pixel> neigh;
        int  filter[3] = {-1, 0, 1};
        for(const int &i : filter){
            for(const int &j : filter){
                if ((x+i) < 0 or (x+i) >= img_height){
                    continue;
                }
                if ((j+y) < 0 or (j+y) >= img_width){
                    continue;
                }
                if ((i+x) == x and (j+y) == y){
                    continue;
                }
                int index_next = img_width*(x+i) + (y+j);
                if((int)depth_img_row_vector.size()>index_next){
                        cv::Scalar intensity = depth_img_row_vector.at(index_next);
                        Pixel selected_item = {index_next, (double)intensity[0]};
                        result.push_back(selected_item);
                }
            }
        }
        return;
    } 

    void LocalMaximaFilter::persistence(const cv::Mat& img, cv::Mat& filtered_image, std::map<int, int>& detected_indics){
        
        int h = img.rows;
        int w = img.cols;
        img_width = w;
        img_height = h;
        uf.set_max_row(w);
        depth_img_row_vector.clear();
        if (img.isContinuous()) {
            depth_img_row_vector.assign((double*)img.data, (double*)img.data + img.total());
        } else {
            for (int i = 0; i < img.rows; ++i) {
                depth_img_row_vector.insert(depth_img_row_vector.end(), img.ptr<double>(i)
                    , img.ptr<double>(i)+img.cols);
            }
        }

        // std::vector<Pixel> indices;
        // for(int i=0; i<h; i++){
        //     for(int j=0; j<w; j++){
        //         int index_val = w*i + j;
        //         cv::Scalar intensity = depth_img_row_vector.at(index_val);
        //         Pixel index_ = {index_val, intensity[0]};
        //         indices.push_back(index_);
        //     }
        // }

        // Get indices orderd by value from high to low
        // __gnu_parallel::sort(indices.rbegin(), indices.rend(),
        //  [&](const Pixel a, const Pixel b){
        //      return a.value < b.value; 
        //  });

        auto cmp = [](Pixel l, Pixel r) { return l.value < r.value;};
        std::priority_queue<Pixel, std::vector<Pixel>, decltype(cmp)> indices_queue(cmp);
        std::vector<Pixel> indices;
        for(int i=0; i<h; i++){
            for(int j=0; j<w; j++){
                cv::Scalar intensity = img.at<double>(i, j);
                int index_val = w*i + j;
                Pixel index_ = {index_val, intensity[0]};
                indices_queue.push(index_);
            }
        }

        while(!indices_queue.empty()) {
            indices.push_back(indices_queue.top());
            indices_queue.pop();
        }

        std::map<int, groups> group0;
        int k = 0;
        for (const auto& p : indices){
            double v = p.value;
            int index_p = p.index;
            std::vector<std::tuple<double,Pixel>> nc;
            std::vector<Pixel> item_list;
            iter_neighbors(p, item_list);
            auto cmp_pixel = [](Pixel lhs, Pixel rhs) 
            { 
                return lhs.value < rhs.value;
            };
            std::set<Pixel, decltype(cmp_pixel)> ni(cmp_pixel);
            for(const Pixel& _object : item_list){
                int index_object = _object.index;
                if (uf.is_contains(index_object)){
                        auto iy = uf.get_items(_object);
                        ni.insert(iy);
                }
            }

            // int couuu=0;
            // for(auto ancesttor : ni){
            //     couuu++;
            //     std::tuple<double, Pixel> __item(ancesttor.value, ancesttor);
            //     nc.push_back(__item); 
            // }

            auto cmp_ni = [](std::tuple<double, Pixel> l, std::tuple<double, Pixel> r) 
            { return std::get<0>(l) > std::get<0>(r);};

            std::priority_queue<std::tuple<double, Pixel>
            , std::vector<std::tuple<double, Pixel>>, decltype(cmp_ni)> ni_queue(cmp_ni);

            int couuu=0;
            for(auto ancesttor : ni){
                couuu++;
                std::tuple<double, Pixel> __item(ancesttor.value, ancesttor);
                ni_queue.push(__item); 
            }
            while(!ni_queue.empty()) {
                nc.push_back(ni_queue.top());
                ni_queue.pop();
            }

            // __gnu_parallel::sort(nc.rbegin(), nc.rend(),
            //     [](const std::tuple<double, Pixel> a, const std::tuple<double, Pixel> b){
            //         return std::get<0>(a) > std::get<0>(b); 
            //     });

            if(k == 0){
                groups gr;
                gr.p1 = v;
                gr.p2 = v;
                gr.p3 = {0, 0.0};
                group0[index_p]= gr;
            }

            uf.add(p, -k, index_p);
            if(nc.size() > 0){
                std::vector<Pixel> indexes;
                indexes.push_back(std::get<1>(nc[0]));
                indexes.push_back(p);
                uf.union_f(indexes);
                for(std::size_t l=1; l<nc.size(); ++l){
                    auto bl = std::get<0>(nc[l]);
                    auto q = std::get<1>(nc[l]);
                    int index_q = q.index;
                    if (uf.is_contains(index_q)){
                            auto corresponding_value = uf.get_items(q);
                            int index_corresponding_value = corresponding_value.index;
                            if(group0.find(index_corresponding_value) == group0.end()){
                                groups gr;
                                gr.p1 = bl;
                                gr.p2 = bl-v;
                                gr.p3 = p;
                                group0[index_corresponding_value] = gr; 
                                std::vector<Pixel> _item_list;
                                int y_index = index_corresponding_value%img_width;
                                detected_indics[y_index] = y_index;
                                iter_neighbors(corresponding_value, _item_list);
                                for(auto const &_object : item_list){
                                    applyBilateralFilter(filtered_image, _object.index, 3, 1.2, 1.2);
                                }
                                iter_neighbors(p, _item_list);
                                for(const auto& _object : item_list){
                                    applyBilateralFilter(filtered_image, _object.index, 3, 1.2, 1.2);
                                }
                            }
                            indexes[1] = corresponding_value;
                            uf.union_f(indexes);
                    }
                }
            }
            k+=1;
        }   
        std::cout<< "==============================" << group0.size() << std::endl;
        return;
    }

    void LocalMaximaFilter::persistence_and_save_data(const cv::Mat& img, cv::Mat& filtered_image, int indexi){
        int h = img.rows;
        int w = img.cols;
        img_width = w;
        img_height = h;
        uf.set_max_row(w);
        depth_img_row_vector.clear();
        if (img.isContinuous()) {
            depth_img_row_vector.assign((double*)img.data, (double*)img.data + img.total());
        } else {
            for (int i = 0; i < img.rows; ++i) {
                depth_img_row_vector.insert(depth_img_row_vector.end(), img.ptr<double>(i)
                    , img.ptr<double>(i)+img.cols);
            }
        }

        // std::vector<Pixel> indices;
        // for(int i=0; i<h; i++){
        //     for(int j=0; j<w; j++){
        //         int index_val = w*i + j;
        //         cv::Scalar intensity = depth_img_row_vector.at(index_val);
        //         Pixel index_ = {index_val, intensity[0]};
        //         indices.push_back(index_);
        //     }
        // }

        // Get indices orderd by value from high to low
        // __gnu_parallel::sort(indices.rbegin(), indices.rend(),
        //  [&](const Pixel a, const Pixel b){
        //      return a.value < b.value; 
        //  });

        auto cmp = [](Pixel l, Pixel r) { return l.value < r.value;};
        std::priority_queue<Pixel, std::vector<Pixel>, decltype(cmp)> indices_queue(cmp);
        std::vector<Pixel> indices;
        for(int i=0; i<h; i++){
            for(int j=0; j<w; j++){
                cv::Scalar intensity = img.at<double>(i, j);
                int index_val = w*i + j;
                Pixel index_ = {index_val, intensity[0]};
                indices_queue.push(index_);
            }
        }

        while(!indices_queue.empty()) {
            indices.push_back(indices_queue.top());
            indices_queue.pop();
        }

        std::map<int, groups> group0;
        int k = 0;
        for (const auto& p : indices){
            double v = p.value;
            int index_p = p.index;
            std::vector<std::tuple<double,Pixel>> nc;
            std::vector<Pixel> item_list;
            iter_neighbors(p, item_list);
            auto cmp_pixel = [](Pixel lhs, Pixel rhs) 
            { 
                return lhs.value < rhs.value;
            };
            std::set<Pixel, decltype(cmp_pixel)> ni(cmp_pixel);
            for(const Pixel& _object : item_list){
                int index_object = _object.index;
                if (uf.is_contains(index_object)){
                        auto iy = uf.get_items(_object);
                        ni.insert(iy);
                }
            }

            // int couuu=0;
            // for(auto ancesttor : ni){
            //     couuu++;
            //     std::tuple<double, Pixel> __item(ancesttor.value, ancesttor);
            //     nc.push_back(__item); 
            // }

            auto cmp_ni = [](std::tuple<double, Pixel> l, std::tuple<double, Pixel> r) 
            { return std::get<0>(l) > std::get<0>(r);};

            std::priority_queue<std::tuple<double, Pixel>
            , std::vector<std::tuple<double, Pixel>>, decltype(cmp_ni)> ni_queue(cmp_ni);

            int couuu=0;
            for(auto ancesttor : ni){
                couuu++;
                std::tuple<double, Pixel> __item(ancesttor.value, ancesttor);
                ni_queue.push(__item); 
            }
            while(!ni_queue.empty()) {
                nc.push_back(ni_queue.top());
                ni_queue.pop();
            }

            // __gnu_parallel::sort(nc.rbegin(), nc.rend(),
            //     [](const std::tuple<double, Pixel> a, const std::tuple<double, Pixel> b){
            //         return std::get<0>(a) > std::get<0>(b); 
            //     });

            if(k == 0){
                groups gr;
                gr.p1 = v;
                gr.p2 = v;
                gr.p3 = {0, 0.0};
                group0[index_p]= gr;
            }

            uf.add(p, -k, index_p);
            if(nc.size() > 0){
                std::vector<Pixel> indexes;
                indexes.push_back(std::get<1>(nc[0]));
                indexes.push_back(p);
                uf.union_f(indexes);
                for(std::size_t l=1; l<nc.size(); ++l){
                    auto bl = std::get<0>(nc[l]);
                    auto q = std::get<1>(nc[l]);
                    int index_q = q.index;
                    if (uf.is_contains(index_q)){
                            auto corresponding_value = uf.get_items(q);
                            int index_corresponding_value = corresponding_value.index;
                            if(group0.find(index_corresponding_value) == group0.end()){
                                groups gr;
                                gr.p1 = bl;
                                gr.p2 = bl-v;
                                gr.p3 = p;
                                group0[index_corresponding_value] = gr; 
                                std::vector<Pixel> _item_list;
                                iter_neighbors(corresponding_value, _item_list);
                                for(auto const &_object : item_list){
                                    applyBilateralFilter(filtered_image, _object.index, 3, 1.2, 1.2);
                                }
                                iter_neighbors(p, _item_list);
                                for(const auto& _object : item_list){
                                    applyBilateralFilter(filtered_image, _object.index, 3, 1.2, 1.2);
                                }
                            }
                            indexes[1] = corresponding_value;
                            uf.union_f(indexes);
                        }
                }
            }
            k+=1;
        }   
        std::cout<< "==============================" << group0.size() << std::endl;

        std::vector<std::tuple<int, int, double, double, double, int, int, double>> groupn;
        for (auto const k : group0){
            auto key = k.first;
            auto val = k.second;
            auto val_k = val.p3;
            std::tuple<int, int, double, double, double, int,
             int, double> m(key/img_width, key%img_width
             , val.p2, val.p1, val.p2
             , val_k.index/img_width, val_k.index%img_width, val_k.value);
            groupn.push_back(m);
        }

        std::sort(groupn.rbegin(), groupn.rend(),
                [](const std::tuple<int, int, double, double, double, int, int, double> a
                , const std::tuple<int, int, double, double, double, int, int, double> b){
                    return std::get<2>(a) > std::get<2>(b); 
                });
        
        std::vector<double> np_mat;
        int groupn_size = groupn.size();
        for(auto const vals : groupn){
            np_mat.push_back((double)std::get<0>(vals));  
            np_mat.push_back((double)std::get<1>(vals));  
            np_mat.push_back((double)std::get<2>(vals));  
            np_mat.push_back((double)std::get<3>(vals));
            np_mat.push_back((double)std::get<4>(vals));  
            np_mat.push_back((double)std::get<5>(vals));  
            np_mat.push_back((double)std::get<6>(vals));  
            np_mat.push_back((double)std::get<7>(vals));   
        }
        
        std::string location = "/dataset/images/result/10/local_maxima_" + std::to_string(indexi) + ".npy";
        cnpy::npy_save(location ,&np_mat[0],{(unsigned int)1, (unsigned int)groupn_size, (unsigned int)8},"w");
        return;
    }



    void LocalMaximaFilter::print_tuple(std::tuple<int, int> object){
        std::cout<< "( " << std::get<0>(object) << " , " << std::get<1>(object) << ")" << std::endl;
    }

    double LocalMaximaFilter::distance(int x, int y, int i, int j) {
        return double(sqrt(pow(x - i, 2) + pow(y - j, 2)));
    }

    double LocalMaximaFilter::gaussian(double x, double sigma) {
        return exp(-(pow(x, 2))/(2 * pow(sigma, 2))) / (2 * CV_PI * pow(sigma, 2));
    }

    void LocalMaximaFilter::applyBilateralFilter(cv::Mat& filteredImage, int source_index, int diameter, double sigmaI, double sigmaS) {
        double iFiltered = 0;
        double wP = 0;
        int neighbor_x = 0;
        int neighbor_y = 0;
        int neighbor_index = 0;
        int half = diameter / 2;
        int x_index = source_index/img_width;
        int y_index = source_index%img_width;
       
        for(int i = 0; i < diameter; i++) {
            for(int j = 0; j < diameter; j++) {
                neighbor_x = x_index - (half - i);
                neighbor_y = y_index - (half - j);
                if (neighbor_y < 0 or neighbor_y >= img_width){
                continue;
                }
                if (neighbor_x < 0 or neighbor_x >= img_height){
                    continue;
                }
                neighbor_index = img_width*neighbor_x + neighbor_y;
                // double gi = gaussian(source.at<double>(neighbor_x, neighbor_y) - source.at<double>(x_index, y_index), sigmaI);
                // double gs = gaussian(distance(x_index, y_index, neighbor_x, neighbor_y), sigmaS);
                double gi = gaussian(depth_img_row_vector.at(neighbor_index) - depth_img_row_vector.at(source_index), sigmaI);
                double gs = gaussian(distance(x_index, y_index, neighbor_x, neighbor_y), sigmaS);

                double w_p = gi * gs;
                // double w_p = gi;
                iFiltered += depth_img_row_vector.at(neighbor_index) * w_p;
                wP += wP + w_p;
            }
        }
        if(wP >0){
            iFiltered = iFiltered / wP;
        }
        // filteredImage.at<double>(x_index, y_index) = iFiltered + depth_img_row_vector.at(neighbor_index);
        filteredImage.at<double>(x_index, y_index) = iFiltered + depth_img_row_vector.at(source_index);
    }
}
}



