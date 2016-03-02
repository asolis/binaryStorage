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


#ifndef __OPENCV_CORE_H5PERSISTENCE_HPP__
#define __OPENCV_CORE_H5PERSISTENCE_HPP__

#include <map>
#include <iostream>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "H5Cpp.h"

namespace cv
{
    class NodeIterator;
    class StorageNode;

    // Main class for HDF5 FileStorage
    // Usage same as FileStorage
    class Storage
    {
    private:
        bool opened;

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
        virtual std::string fullName(const std::string &node) const;
        virtual void nodeWritten();
        virtual void startScope(const std::string &name, int type);
        virtual void endScope();

        int  state;
        std::string elname;
        std::vector<char> structs;
        std::string root;

        Ptr<H5::H5File> _bfs;
        std::vector<std::string> groups;
        std::vector<size_t> counter;

        static inline bool cv_isalpha(char c)
        {
            return ('a' <= c && c <= 'z') ||
                   ('A' <= c && c <= 'Z');
        }
    };
    // Class to represente a Node
    // Usage same as FileNode
    class StorageNode
    {
    public:
        int _type;
        Ptr<H5::H5File> _fn;
        std::string     _name;
        std::vector<std::string> _sub;

        const static std::string TYPE;
        const static std::string ROWS;
        const static std::string COLS;
        const static std::string MTYPE;
        const static size_t BUFFER  = 100;

        enum Type
        {
            NONE      = 0, //!< empty node
            INT       = 1, //!< an integer
            REAL      = 2, //!< floating-point number
            FLOAT     = REAL, //!< synonym or REAL
            STR       = 3, //!< text string in UTF-8 encoding
            STRING    = STR, //!< synonym for STR
            REF       = 4, //!< integer of size size_t. Typically used for storing complex dynamic structures where some elements reference the others
            SEQ       = 5, //!< sequence
            MAP       = 6, //!< mapping
            TYPE_MASK = 7,
            FLOW      = 8,  //!< compact representation of a sequence or mapping.
            USER      = 16, //!< a registered object (e.g. a matrix)
            EMPTY     = 32, //!< empty structure (sequence or mapping)
            NAMED     = 64  //!< the node has a name (i.e. it is element of a mapping)
        };

        StorageNode();
        StorageNode(const Ptr<H5::H5File> &f, const std::string &name);
        virtual StorageNode operator[](const std::string &nodename) const;
        virtual StorageNode operator[](const char *nodename) const;
        virtual StorageNode operator[](int i)const;

        virtual bool empty()   const;
        virtual int  type()   const;
        virtual bool isNone() const;
        virtual bool isSeq()  const;
        virtual bool isMap()  const;
        virtual bool isInt()  const;
        virtual bool isReal() const;
        virtual bool isString() const;

        virtual std::string name() const;
        virtual size_t size() const;

        virtual operator int() const;
        virtual operator float() const;
        virtual operator double() const;
        virtual operator std::string() const;

        virtual NodeIterator begin() const;
        virtual NodeIterator end() const;
    };

    // Class to iterate over a StorageNode
    // Usage same as FileNodeIterator
    class NodeIterator
    {
    private:
        size_t _pos;
        StorageNode _node;
    public:
        NodeIterator();
        NodeIterator(const StorageNode &node, size_t pos = 0);
        //! returns the currently observed element
        virtual StorageNode operator *() const;
        //! accesses the currently observed element methods
        virtual StorageNode operator ->() const;
        //! moves iterator to the next node
        virtual NodeIterator& operator ++ ();
        //! moves iterator to the next node
        virtual NodeIterator operator ++ (int);
        //! moves iterator to the previous node
        virtual NodeIterator& operator -- ();
        //! moves iterator to the previous node
        virtual NodeIterator operator -- (int);
        //! moves iterator forward by the specified offset (possibly negative)
        virtual NodeIterator& operator += (int ofs);
        //! moves iterator backward by the specified offset (possibly negative)
        virtual NodeIterator& operator -= (int ofs);

        //! compares if two nodes are pointing to the same element
        virtual bool operator == (const NodeIterator& rhs);
        //! compares if two nodes are not pointing to the same element
        virtual bool operator != (const NodeIterator& rhs);
        //! pointer aritmetic between two nodes
        virtual ptrdiff_t operator- (const NodeIterator& rhs);
        //! compares the order of two iterator positions
        virtual bool operator < (const NodeIterator& rhs);

    };


    //HDF5 Attribute functions
    bool getAttribute(H5::H5Location &node, const H5::DataType &type,
                      const std::string &name, void *value);
    void setAttribute(H5::H5Location &node, const H5::DataType &type,
                      const std::string &name, const void *value);
    void setStrAttribute(H5::H5Location &node,
                         const std::string &name, const char *value);
    void setStrAttribute(H5::H5Location &node,
                         const std::string &name, const std::string &value);
    void setIntAttribute(H5::H5Location &node,
                         const std::string &name, const int &num);
    bool getNodeType(H5::H5Location &node, int &value);
    void setNodeType(H5::H5Location &node, int type);

    //HDF5 Group functions
    void setGroupAttribute(H5::H5File &file, const std::string &gname,
                           const std::string &attr, int value);
    void createGroup(H5::H5File &file, const std::string &name, int type);
    //HDF5 helper functions to list subnodes
    bool sortSequenceDatasets(const std::string &a, const std::string &b);
    herr_t readH5NodeInfo(hid_t loc_id, const char *name, const H5L_info_t *linfo, void *opdata);
    void listSubnodes(H5::H5Location &node, std::vector<std::string> &subnodes);
    void getNode(H5::H5File &file, const std::string &name, H5::H5Location &node);

    //HDF5 Write basic and OpenCV types
    void writeDataset1D(H5::H5File &file, const H5::DataType &type,
                        const std::string &name, const void *data,
                        const std::map<std::string,int> &attr, size_t count);

    void writeDataset(H5::H5File &file, const std::string &name, int data);
    void writeDataset(H5::H5File &file, const std::string &name, float data);
    void writeDataset(H5::H5File &file, const std::string &name, double data);
    void writeDataset(H5::H5File &file, const std::string &name, const std::string &data);
    void writeDataset(H5::H5File &file, const std::string &name, const cv::Mat &data);
    void writeDataset(H5::H5File &file, const std::string &name, const cv::SparseMat &data);
    void writeDataset(H5::H5File &file, const std::string &name, const std::vector<int> &data);
    void writeDataset(H5::H5File &file, const std::string &name, const std::vector<float> &data);
    void writeDataset(H5::H5File &file, const std::string &name, const std::vector<double> &data);
    void writeDataset(H5::H5File &file, const std::string &name, const std::vector<std::string> &vec);
    void writeDataset(H5::H5File &file, const std::string &name, const std::vector<cv::KeyPoint> &ks);
    void writeDataset(H5::H5File &file, const std::string &name, const std::vector<cv::DMatch> &ks);
    void writeDataset(H5::H5File &file, const std::string &name, const cv::Range& _data);

    template<typename _Tp>
    inline void writeDataset(H5::H5File &file, const std::string &name, const Point_<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        _tmp.push_back(_data.x);
        _tmp.push_back(_data.y);
        writeDataset(file, name, _tmp);
    }
    template<typename _Tp>
    inline void writeDataset(H5::H5File &file, const std::string &name, const Point3_<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        _tmp.push_back(_data.x);
        _tmp.push_back(_data.y);
        _tmp.push_back(_data.z);
        writeDataset(file, name, _tmp);
    }
    template<typename _Tp>
    inline void writeDataset(H5::H5File &file, const std::string &name, const Size_<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        _tmp.push_back(_data.width);
        _tmp.push_back(_data.height);
        writeDataset(file, name, _tmp);
    }
    template<typename _Tp>
    inline void writeDataset(H5::H5File &file, const std::string &name, const Complex<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        _tmp.push_back(_data.re);
        _tmp.push_back(_data.im);
        writeDataset(file, name, _tmp);
    }
    template<typename _Tp>
    inline void writeDataset(H5::H5File &file, const std::string &name,  const Rect_<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        _tmp.push_back(_data.x);
        _tmp.push_back(_data.y);
        _tmp.push_back(_data.width);
        _tmp.push_back(_data.height);
        writeDataset(file, name, _tmp);
    }
    template<typename _Tp>
    inline void writeDataset(H5::H5File &file, const std::string &name, const Scalar_<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        _tmp.push_back(_data.val[0]);
        _tmp.push_back(_data.val[1]);
        _tmp.push_back(_data.val[2]);
        _tmp.push_back(_data.val[3]);
        writeDataset(file, name, _tmp);
    }

    //HDF5 Read basic and OpenCV types
    template<typename _Tp>
    bool readDataset1D(const H5::H5File &file, const std::string &name, std::vector<_Tp> &data)
    {
        H5::DataSet dataset = file.openDataSet(name);
        H5::DataSpace dataspace = dataset.getSpace();
        hsize_t dims_out[1];
        int rank = dataspace.getSimpleExtentDims( dims_out, NULL);

        int _type;
        bool read = getNodeType(dataset,  _type);
        read &= (_type == StorageNode::SEQ);
        read &= (rank  == 1);

        if (!read)
            return read;
        data.resize(dims_out[0]);
        dataset.read(data.data(), dataset.getDataType());
        return true;
    }
    bool readDataset(const H5::H5File &file, const std::string &name, int type, void *data);
    bool readDataset(const H5::H5File &file, const std::string &name, int &data);
    bool readDataset(const H5::H5File &file, const std::string &name, float &data);
    bool readDataset(const H5::H5File &file, const std::string &name, double &data);
    bool readDataset(const H5::H5File &file, const std::string &name, std::string &data);
    bool readDataset(const H5::H5File &file, const std::string &name, Mat &data);
    bool readDataset(const H5::H5File &file, const std::string &name, SparseMat &data);
    bool readDataset(const H5::H5File &file, const std::string &name, std::vector<int> &data);
    bool readDataset(const H5::H5File &file, const std::string &name, std::vector<float> &data);
    bool readDataset(const H5::H5File &file, const std::string &name, std::vector<double> &data);
    bool readDataset(const H5::H5File &file, const std::string &name, std::vector<std::string> &data);
    bool readDataset(const H5::H5File &file, const std::string &name, std::vector<cv::KeyPoint> &ks);
    bool readDataset(const H5::H5File &file, const std::string &name, std::vector<cv::DMatch> &dm);
    bool readDataset(const H5::H5File &file, const std::string &name, Range& _data);

    template<typename _Tp>
    inline bool readDataset(H5::H5File &file, const std::string &name, Point_<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        bool read = readDataset1D<_Tp>(file, name, _tmp);
        if (!read)
            return read;
        _data.x = _tmp[0];
        _data.y = _tmp[1];
        return true;
    }
    template<typename _Tp>
    inline bool readDataset(H5::H5File &file, const std::string &name, Point3_<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        bool read = readDataset1D<_Tp>(file, name, _tmp);
        if (!read)
            return read;
        _data.x = _tmp[0];
        _data.y = _tmp[1];
        _data.y = _tmp[2];
        return true;
    }
    template<typename _Tp>
    inline bool readDataset(H5::H5File &file, const std::string &name, Size_<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        bool read = readDataset1D<_Tp>(file, name, _tmp);
        if (!read)
            return read;
        _data.width  = _tmp[0];
        _data.height = _tmp[1];
        return true;
    }
    template<typename _Tp>
    inline bool readDataset(H5::H5File &file, const std::string &name, Complex<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        bool read = readDataset1D<_Tp>(file, name, _tmp);
        if (!read)
            return read;
        _data.re  = _tmp[0];
        _data.im = _tmp[1];
        return true;
    }
    template<typename _Tp>
    inline bool readDataset(H5::H5File &file, const std::string &name, Rect_<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        bool read = readDataset1D<_Tp>(file, name, _tmp);
        if (!read)
            return read;
        _data.x  = _tmp[0];
        _data.y = _tmp[1];
        _data.width = _tmp[2];
        _data.height = _tmp[3];
        return true;
    }
    template<typename _Tp>
    inline bool readDataset(H5::H5File &file, const std::string &name, Scalar_<_Tp>& _data)
    {
        std::vector<_Tp> _tmp;
        bool read = readDataset1D<_Tp>(file, name, _tmp);
        if (!read)
            return read;
        _data.val[0] = _tmp[0];
        _data.val[1] = _tmp[1];
        _data.val[2] = _tmp[2];
        _data.val[3] = _tmp[3];
        return true;
    }

    // Storage, StorageNode, NodeIterator Operator and helper functions for each
    // basic and OpenCV types.

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
