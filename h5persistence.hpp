/**************************************************************************************************
 **************************************************************************************************

 BSD 3-Clause License (https://www.tldrlegal.com/l/bsd3)

 Copyright (c) 2016 Andrés Solís Montero <http://www.solism.ca>, All rights reserved.


 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 OF THE POSSIBILITY OF SUCH DAMAGE.

 **************************************************************************************************
 **************************************************************************************************/

#ifndef __OPENCV_CORE_HDF5PERSISTENCE_HPP__
#define __OPENCV_CORE_HDF5PERSISTENCE_HPP__


#include <map>
#include <iostream>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "H5Cpp.h"
#include "storage.hpp"

namespace cv
{

    class Storage
    {
    public:
        enum Mode
        {
            READ        = 0,
            WRITE       = 1, 
            APPEND      = 2
        };
        enum
        {
            UNDEFINED      = 0,
            VALUE_EXPECTED = 1,
            NAME_EXPECTED  = 2,
            INSIDE_MAP     = 4,
            INSIDE_SEQ     = 8
        };

        Storage();
        Storage(const std::string &filename, int flags);
        virtual ~Storage();
        virtual bool open(const std::string &filename, int flags);
        virtual bool isOpened();
        virtual void release();
        virtual cv::StorageNode getFirstTopLevelNode() const;
        virtual cv::StorageNode operator[](const String& nodename) const;
        virtual cv::StorageNode operator[](const char* nodename) const;

    private:
        bool opened;

    public:
        int  state;

        std::string elname;
        std::vector<char> structs;
        std::string root;

        Ptr<H5::H5File> _bfs;
        std::vector<std::string> groups;
        std::vector<size_t> counter;

        virtual std::string fullName(const std::string &node) const
        {
            std::stringstream ss;
            ss << "/";
            for (size_t i = 0; i < groups.size(); i++)
            {
                ss << groups[i] << "/";
            }
            ss << ((node.empty())? std::to_string(counter.back()): node);
            return ss.str();
        }
        virtual void nodeWritten()
        {
            if (state == cv::Storage::VALUE_EXPECTED &&
                elname == "" &&
                groups.size() > 0 )
            {
                counter.back()++;
            }
        }
        virtual void startScope(const std::string &name, int type)
        {
            cv::createGroup(*_bfs, fullName(name), type);
            groups.push_back(((name.empty())? std::to_string(counter.back()): name));
            counter.push_back(0);
        }
        virtual void endScope()
        {
            groups.pop_back();
            counter.pop_back();

        }
    };
    
    void write(Storage &fs, const std::string &name, int _data);
    void write(Storage &fs, const std::string &name, float _data);
    void write(Storage &fs, const std::string &name, double _data);
    void write(Storage &fs, const std::string &name, const std::string &_data);
    void write(Storage &fs, const std::string &name, const cv::Mat &_data);
    void write(Storage &fs, const std::string &name, const cv::SparseMat &_data);
    void write(Storage &fs, const std::string &name, const std::vector<int> &_data);
    void write(Storage &fs, const std::string &name, const std::vector<float> &_data);
    void write(Storage &fs, const std::string &name, const std::vector<double> &_data);
    void write(Storage &fs, const std::string &name, const std::vector<std::string> &_data);
    void write(Storage &fs, const std::string &name, const std::vector<cv::KeyPoint> &_data);
    void write(Storage &fs, const std::string &name, const std::vector<cv::DMatch> &_data);
    void write(Storage &fs, const std::string &name, const cv::Range& _data);

    void read(const StorageNode &fn, int &_data, int default_value);
    void read(const StorageNode &fn, float &_data, float default_value);
    void read(const StorageNode &fn, double &_data, double default_value);
    void read(const StorageNode &fn, std::string &_data, const std::string &default_value);
    void read(const StorageNode &fn, cv::Mat &_data, const cv::Mat &default_value = cv::Mat());
    void read(const StorageNode &fn, cv::SparseMat &_data, const cv::SparseMat &default_value = cv::SparseMat());
    void read(const StorageNode &fn, std::vector<int> &_data, const std::vector<int> &default_value);
    void read(const StorageNode &fn, std::vector<float> &_data, const std::vector<float> &default_value);
    void read(const StorageNode &fn, std::vector<double> &_data, const std::vector<double> &default_value);
    void read(const StorageNode &fn, std::vector<std::string> &_data, const std::vector<std::string> &default_value);
    void read(const StorageNode &fn, std::vector<cv::KeyPoint> &_data, const std::vector<cv::KeyPoint> &default_value);
    void read(const StorageNode &fn, std::vector<cv::DMatch> &_data, const std::vector<cv::DMatch> &default_value);
    void read(const StorageNode &fn, cv::Range& _data, const cv::Range &default_value);


    template<typename _Tp>
    inline void write(Storage &fs, const std::string &name, const Point_<_Tp>& _data)
    {
        cv::writeDataset<_Tp>(*fs._bfs, name, _data);
    }
    template<typename _Tp>
    inline void write(Storage &fs, const std::string &name, const Point3_<_Tp>& _data)
    {
        cv::writeDataset<_Tp>(*fs._bfs, name, _data);
    }
    template<typename _Tp>
    inline void write(Storage &fs, const std::string &name, const Size_<_Tp>& _data)
    {
        cv::writeDataset<_Tp>(*fs._bfs, name, _data);
    }
    template<typename _Tp>
    inline void write(Storage &fs, const std::string &name, const Complex<_Tp>& _data)
    {
        cv::writeDataset<_Tp>(*fs._bfs, name, _data);
    }
    template<typename _Tp>
    inline void write(Storage &fs, const std::string &name,  const Rect_<_Tp>& _data)
    {
        cv::writeDataset<_Tp>(*fs._bfs, name, _data);
    }
    template<typename _Tp>
    inline void write(Storage &fs, const std::string &name, const Scalar_<_Tp>& _data)
    {
        cv::writeDataset<_Tp>(*fs._bfs, name, _data);
    }

    template<typename _Tp>
    inline void read(const StorageNode &fn, Point_<_Tp>& _data, const Point_<_Tp>& default_value = Point_<_Tp>())
    {
        if (!cv::readDataset<_Tp>(*fn._fn, fn._name, _data))
            _data = default_value;
    }
    template<typename _Tp>
    inline void read(const StorageNode &fn, Point3_<_Tp>& _data, const Point3_<_Tp>& default_value = Point3_<_Tp>())
    {
        if (!cv::readDataset<_Tp>(*fn._fn, fn._name, _data))
            _data = default_value;
    }
    template<typename _Tp>
    inline void read(const StorageNode &fn, Size_<_Tp>& _data, const Size_<_Tp>& default_value = Size_<_Tp>())
    {
        if (!cv::readDataset<_Tp>(*fn._fn, fn._name, _data))
            _data = default_value;
    }
    template<typename _Tp>
    inline void read(const StorageNode &fn, Complex<_Tp>& _data, const Complex<_Tp>& default_value = Complex<_Tp>())
    {
        if (!cv::readDataset<_Tp>(*fn._fn, fn._name, _data))
            _data = default_value;
    }
    template<typename _Tp>
    inline void read(const StorageNode &fn, Rect_<_Tp>& _data, const Rect_<_Tp>& default_value = Rect_<_Tp>())
    {
        if (!cv::readDataset<_Tp>(*fn._fn, fn._name, _data))
            _data = default_value;
    }
    template<typename _Tp>
    inline void read(const StorageNode &fn, Scalar_<_Tp>& _data, const Scalar_<_Tp>& default_value = Scalar_<_Tp>())
    {
        if (!cv::readDataset<_Tp>(*fn._fn, fn._name, _data))
            _data = default_value;
    }

    static inline bool cv_isalpha(char c)
    {
        return ('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z');
    }

    cv::Storage& operator << (cv::Storage& fs, const std::string &str);

    inline Storage& operator << (Storage& fs, const char* str)
    {
        return (fs << std::string(str));
    }
    inline Storage& operator << (Storage& fs, char* value)
    {
        return (fs << std::string(value));
    }
    template<typename _Tp>
    inline Storage& operator << (Storage& fs, const _Tp& value)
    {
        if( !fs.isOpened() )
            return fs;
        if( fs.state == Storage::NAME_EXPECTED + Storage::INSIDE_MAP )
            CV_Error(CV_StsError, "No element name has been given" );
        //write( fs, fs.elname, value );
        write(fs, fs.fullName(fs.elname), value);
        fs.nodeWritten();
        if( fs.state & Storage::INSIDE_MAP )
            fs.state = Storage::NAME_EXPECTED +
                       Storage::INSIDE_MAP;
        return fs;
    }

    template<typename T>
    static inline void write(Storage &fs, const std::string &name, const std::vector<T> &data)
    {
        fs << "[" ;
        for (size_t i = 0; i < data.size(); i++)
            fs << data[i];
        fs << "]";
    }
    template<typename T>
    static inline void write(Storage &fs, const std::string &name, const std::map<std::string,T> &data)
    {
        fs << "{";
        for(typename std::map<std::string, T>::const_iterator it = data.begin();
            it != data.end(); it++)
            fs << it->first << it->second;
        fs << "}";
    }


    template<typename _Tp> static inline
    cv::NodeIterator& operator >> (cv::NodeIterator& it, _Tp& value)
    {
        read( *it, value, _Tp());
        return ++it;
    }

//    /** @brief Reads data from a file storage.
//     */
//    template<typename _Tp> static inline
//    cv::NodeIterator& operator >> (cv::NodeIterator& it, std::vector<_Tp>& vec)
//    {
//        cv::NodeIterator it2 = it;
//        _Tp element;
//        read(*it, element, _Tp());
//
//        while (it2 != it++)
//        {
//            read(*it, element, _Tp());
//            it2 = it;
//        }
//        return it;
//    }

    /** @brief Reads data from a file storage.
     */
    template<typename _Tp> static inline
    void operator >> (const cv::StorageNode& n, _Tp& value)
    {
        read( n, value, _Tp());
    }

//    /** @brief Reads data from a file storage.
//     */
//    template<typename _Tp> static inline
//    void operator >> (const cv::StorageNode& n, std::vector<_Tp>& vec)
//    {
//        cv::NodeIterator it = n.begin();
//        it >> vec;
//    }

}


#endif