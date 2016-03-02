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
#include <algorithm>
#include "h5persistence.hpp"

/***** StorageNode members and functions ******/
const  std::string cv::StorageNode::TYPE = "type";
const  std::string cv::StorageNode::ROWS = "rows";
const  std::string cv::StorageNode::COLS = "cols";
const  std::string cv::StorageNode::MTYPE= "mTyp";

cv::StorageNode::StorageNode():
_type(StorageNode::NONE), _fn(Ptr<H5::H5File>()), _name(""), _sub(0)
{}

cv::StorageNode::StorageNode(const Ptr<H5::H5File> &f, const std::string &name):
_fn(f), _name(name)
{
    H5::H5Location *loc = NULL;
    try
    {
        H5::Exception::dontPrint();
        loc = new H5::DataSet(f->openDataSet(name));
    }
    catch( H5::FileIException not_found_error)
    {
        try
        {
            H5::Exception::dontPrint();
            loc = new H5::Group(f->openGroup(name));
        }
        catch( H5::FileIException not_found_error)
        {

        }
    }
    if (loc)
    {
        bool read = cv::getNodeType(*loc, _type);
        if (!read) _type = StorageNode::NONE;
        cv::listSubnodes(*loc, _sub);
    }
    else
        _type = StorageNode::NONE;
}

cv::StorageNode cv::StorageNode::operator[](const std::string &nodename) const
{
    std::vector<std::string>::const_iterator it;
    it = std::find(_sub.begin(), _sub.end(), nodename);
    if (it != _sub.end())
    {
        std::string fName =  (_name.back() != '/') ?
        (_name + "/" + nodename) :
        (_name + nodename);
        return StorageNode(_fn, fName);
    }
    else
        StorageNode();
}

cv::StorageNode cv::StorageNode::operator[](const char *nodename) const
{
    return this->operator[](std::string(nodename));
}

cv::StorageNode cv::StorageNode::operator[](int i) const
{
    if ( i >= 0 && i < _sub.size() )
    {
        std::string fName =  (_name.back() != '/') ?
        (_name + "/" + _sub[i]) :
        (_name + _sub[i]);
        return StorageNode(_fn, fName);
    }
    else
        return StorageNode();
}

int  cv::StorageNode::type()   const
{
    return _type;
}
bool  cv::StorageNode::empty()   const
{
    return type() == StorageNode::NONE;;
}
bool cv::StorageNode::isNone() const
{
    return type() == StorageNode::NONE;
}
bool cv::StorageNode::isSeq()  const
{
    return type() == StorageNode::SEQ;
}
bool cv::StorageNode::isMap()  const
{
    return type() == StorageNode::MAP;
}
bool cv::StorageNode::isInt()  const
{
    return type() == StorageNode::INT;
}
bool cv::StorageNode::isReal() const
{
    return type() == StorageNode::REAL;
}
bool cv::StorageNode::isString() const
{
    return type() == StorageNode::STRING;
}

std::string cv::StorageNode::name() const
{
    return _name;
}
size_t cv::StorageNode::size() const
{
    return _sub.size();
}

cv::StorageNode::operator int() const
{
    int value;
    if (!cv::readDataset(*_fn, _name, value))
        return 0;
    return value;
}
cv::StorageNode::operator float() const
{
    float value;
    if (!cv::readDataset(*_fn, _name, value))
        return 0;
    return value;
}
cv::StorageNode::operator double() const
{
    double value;
    if (!cv::readDataset(*_fn, _name, value))
        return 0;
    return value;
}
cv::StorageNode::operator std::string() const
{
    string value;
    if (!cv::readDataset(*_fn, _name, value))
        return "";
    return value;
}
cv::NodeIterator cv::StorageNode::begin() const
{
    return NodeIterator(*this);
}
cv::NodeIterator cv::StorageNode::end() const
{
    return NodeIterator(*this, _sub.size());
}



/***** NodeIterator members and functions ******/
cv::NodeIterator::NodeIterator():
_pos(0), _node()
{}

cv::NodeIterator::NodeIterator(const StorageNode &node, size_t pos):
_pos(pos), _node(node)
{}

//! returns the currently observed element
cv::StorageNode cv::NodeIterator::operator *() const
{
    if (_pos < _node._sub.size())
    {
        std::string fName =  (_node._name.back() != '/') ?
        (_node._name + "/" + _node._sub[_pos]) :
        (_node._name + _node._sub[_pos]);
        return StorageNode(_node._fn,  fName);
    }
    else
        return StorageNode();
}
//! accesses the currently observed element methods
cv::StorageNode cv::NodeIterator::operator ->() const
{
    return this->operator*();
}

//! moves iterator to the next node
cv::NodeIterator& cv::NodeIterator::operator ++ ()
{
    ++_pos;
    return *this;
}
//! moves iterator to the next node
cv::NodeIterator cv::NodeIterator::operator ++ (int)
{
    NodeIterator it = *this;
    ++(*this);
    return it;
}
//! moves iterator to the previous node
cv::NodeIterator& cv::NodeIterator::operator -- ()
{
    --_pos;
    return *this;
}
//! moves iterator to the previous node
cv::NodeIterator cv::NodeIterator::operator -- (int)
{
    NodeIterator it = *this;
    --(*this);
    return it;
}
//! moves iterator forward by the specified offset (possibly negative)
cv::NodeIterator& cv::NodeIterator::operator += (int ofs)
{
    _pos += ofs;
    return *this;
}
//! moves iterator backward by the specified offset (possibly negative)
cv::NodeIterator& cv::NodeIterator::operator -= (int ofs)
{
    _pos -= ofs;
    return *this;
}

bool cv::NodeIterator::operator == (const NodeIterator& rhs)
{
    return this->_pos == rhs._pos &&
    this->_node._fn == rhs._node._fn &&
    this->_node._name == rhs._node._name ;
}

bool cv::NodeIterator::operator != (const NodeIterator& rhs)
{
    return !(*this == rhs);
}

ptrdiff_t cv::NodeIterator::operator- (const NodeIterator& rhs)
{
    return this->_pos - rhs._pos;
}

bool cv::NodeIterator::operator < (const NodeIterator& rhs)
{
    return this->_pos < rhs._pos;
}

/***** Storage members and functions ******/
cv::Storage::Storage():
opened(false), state(UNDEFINED), root("/")
{}

cv::Storage::Storage(const std::string &filename, int flags):
opened(false), state(UNDEFINED), root("/")
{
    open(filename, flags);
}
bool cv::Storage::open(const std::string &filename, int flags)
{
    opened = false;
    try
    {
        if (flags == READ)
            _bfs = new H5::H5File(filename, H5F_ACC_RDONLY );
        else if (flags == WRITE)
            _bfs = new H5::H5File(filename, H5F_ACC_TRUNC );
        else if (flags == APPEND)
            _bfs = new H5::H5File(filename, H5F_ACC_RDWR);

        opened = true;
    }
    catch( H5::FileIException error )
    {
        opened =false;
    }

    state = (opened)? NAME_EXPECTED + INSIDE_MAP : UNDEFINED;
    counter.push_back(0);
    return opened;
}
cv::Storage::~Storage()
{
    release();
}
bool cv::Storage::isOpened()
{
    return opened;
}
void cv::Storage::release()
{
    if (!isOpened())
        return;
    state = UNDEFINED;
}

cv::StorageNode cv::Storage::getFirstTopLevelNode() const
{
    return StorageNode(_bfs, "/");
}

cv::StorageNode cv::Storage::operator[](const String& nodename) const
{
    return StorageNode(_bfs, "/" + nodename);
}
cv::StorageNode cv::Storage::operator[](const char* nodename) const
{
    return this->operator[](std::string(nodename));
}
cv::Storage& cv::operator<< (cv::Storage& fs, const std::string &str)
{
    const char* _str = str.c_str();

    if( !fs.isOpened() || !_str )
        return fs;

    if( *_str == '}' || *_str == ']' )
    {
        if( fs.structs.empty() )
            CV_Error_( CV_StsError, ("Extra closing '%c'", *_str) );

        if( (*_str == ']' ? '[' : '{') != fs.structs.back() )
            CV_Error_( CV_StsError,
                      ("The closing '%c' does not match the opening '%c'",
                       *_str, fs.structs.back()));

        fs.structs.pop_back();
        fs.state = (fs.structs.empty() ||
                    fs.structs.back() == '{' )?
        cv::Storage::INSIDE_MAP + cv::Storage::NAME_EXPECTED :
        cv::Storage::VALUE_EXPECTED;

        //cvEndWriteStruct
        fs.endScope();

        fs.elname = "";
    }
    else if( fs.state == cv::Storage::NAME_EXPECTED +
            cv::Storage::INSIDE_MAP )
    {

        if( !cv::cv_isalpha(*_str) )
            CV_Error_( CV_StsError, ("Incorrect element name %s", _str) );
        fs.elname = str;
        fs.state = cv::Storage::VALUE_EXPECTED +
        cv::Storage::INSIDE_MAP;
    }
    else if( (fs.state & 3) == cv::Storage::VALUE_EXPECTED )
    {
        if( *_str == '{' || *_str == '[' )
        {
            fs.structs.push_back(*_str);
            int flags = (*_str++ == '{')? CV_NODE_MAP : CV_NODE_SEQ;

            fs.state = (flags == CV_NODE_MAP) ?
            cv::Storage::INSIDE_MAP + cv::Storage::NAME_EXPECTED :
            cv::Storage::VALUE_EXPECTED;

            if( *_str == ':' )
            {
                flags |= CV_NODE_FLOW;
                _str++;
            }
            //cvStartWriteStruct
            fs.startScope(fs.elname, (flags == CV_NODE_MAP)? cv::StorageNode::MAP :
                          cv::StorageNode::SEQ);
            fs.elname = "";
        }
        else
        {
            std::string _tmp_ = (_str[0] == '\\' &&
                                 (_str[1] == '{' || _str[1] == '}' ||
                                  _str[1] == '[' || _str[1] == ']')) ?
            std::string(_str+1) : str;

            write( fs, fs.fullName(fs.elname), _tmp_);

            fs.nodeWritten();

            if( fs.state == cv::Storage::INSIDE_MAP +
               cv::Storage::VALUE_EXPECTED )
                fs.state = cv::Storage::INSIDE_MAP +
                cv::Storage::NAME_EXPECTED;
        }
    }
    else
        CV_Error( CV_StsError, "Invalid fs.state" );
    return fs;
}

/***Writting H5***/
void cv::write(cv::Storage &fs, const std::string &name, int _data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, float _data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, double _data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, const std::string &_data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, const cv::Mat &_data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, const cv::SparseMat &_data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, const std::vector<int> &_data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, const std::vector<float> &_data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, const std::vector<double> &_data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, const std::vector<std::string> &_data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, const std::vector<cv::KeyPoint> &_data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, const std::vector<cv::DMatch> &_data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}
void cv::write(cv::Storage &fs, const std::string &name, const cv::Range& _data)
{
    cv::writeDataset(*fs._bfs, name, _data);
}

void cv::read(const cv::StorageNode &fn, int &_data, int default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, float &_data, float default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, double &_data, double default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, std::string &_data, const std::string &default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, cv::Mat &_data, const cv::Mat &default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, cv::SparseMat &_data, const cv::SparseMat &default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, std::vector<int> &_data, const std::vector<int> &default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, std::vector<float> &_data, const std::vector<float> &default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, std::vector<double> &_data, const std::vector<double> &default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, std::vector<std::string> &_data, const std::vector<std::string> &default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, std::vector<cv::KeyPoint> &_data, const std::vector<cv::KeyPoint> &default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, std::vector<cv::DMatch> &_data, const std::vector<cv::DMatch> &default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}
void cv::read(const cv::StorageNode &fn, cv::Range& _data, const cv::Range &default_value)
{
    if (!readDataset(*fn._fn, fn._name, _data))
        _data = default_value;
}


//Attributes
bool cv::getAttribute(H5::H5Location &node, const H5::DataType &type, const std::string &name, void *value)
{
    hsize_t dims[] = {1};
    H5::DataSpace attSpace(1, dims);
    if (!node.attrExists(name))
        return false;
    H5::Attribute attribute = node.openAttribute(name);
    attribute.read(type, value);
    return true;
}
void cv::setAttribute(H5::H5Location &node, const H5::DataType &type, const std::string &name, const void *value)
{
    hsize_t dims[] = {1};
    H5::DataSpace attSpace(1, dims);
    H5::Attribute attribute = (!node.attrExists(name))?
    node.createAttribute(name, type , attSpace):
    node.openAttribute(name);
    attribute.write(type, value);
}
void cv::setStrAttribute(H5::H5Location &node, const std::string &name, const char *value)
{
    std::vector<const char*> arr_c_str;
    arr_c_str.push_back(value);
    setAttribute(node, H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), name, arr_c_str.data());
}
void cv::setStrAttribute(H5::H5Location &node, const std::string &name, const std::string &value)
{
    setAttribute(node, H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), name, value.c_str());
}
void cv::setIntAttribute(H5::H5Location &node, const std::string &name, const int &num)
{
    setAttribute(node, H5::PredType::NATIVE_INT, name, &num);
}
bool cv::getNodeType(H5::H5Location &node, int &value)
{
    return getAttribute(node, H5::PredType::NATIVE_INT, StorageNode::TYPE, &value);
}
void cv::setNodeType(H5::H5Location &node, int type)
{
    setIntAttribute(node, StorageNode::TYPE,  type);
}


//Groups
void cv::setGroupAttribute(H5::H5File &file, const std::string &gname, const std::string &attr, int value)
{
    H5::Group group(file.openGroup(gname));
    setIntAttribute(group, attr,  value);
}
void cv::createGroup(H5::H5File &file, const std::string &name, int type)
{
    H5::Group group(file.createGroup( name ));
    setNodeType(group, type);
}


//Write Basic types
void cv::writeDataset1D(H5::H5File &file, const H5::DataType &type,
                        const std::string &name, const void *data,
                        const std::map<std::string,int> &attr, size_t count)
{
    hsize_t dims[] = {count};
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dataset(file.createDataSet(name.c_str(), type, dataspace));

    for (std::map<std::string, int>::const_iterator it = attr.begin(); it != attr.end(); it++)
    {
        setIntAttribute(dataset, it->first, it->second);
    }

    dataset.write(data, type);
}
void cv::writeDataset(H5::H5File &file, const std::string &name, int data)
{
    std::map<std::string,int> attr;
    attr.insert(std::make_pair(StorageNode::TYPE, StorageNode::INT));
    writeDataset1D(file, H5::PredType::NATIVE_INT, name, &data, attr, 1);
}
void cv::writeDataset(H5::H5File &file, const std::string &name, float data)
{
    std::map<std::string,int> attr;
    attr.insert(std::make_pair(StorageNode::TYPE, StorageNode::REAL));
    writeDataset1D(file, H5::PredType::NATIVE_FLOAT, name, &data,attr, 1);
}
void cv::writeDataset(H5::H5File &file, const std::string &name, double data)
{
    std::map<std::string,int> attr;
    attr.insert(std::make_pair(StorageNode::TYPE, StorageNode::REAL));
    writeDataset1D(file, H5::PredType::NATIVE_DOUBLE, name, &data, attr, 1);
}
void cv::writeDataset(H5::H5File &file, const std::string &name, const std::string &data)
{
    std::vector<const char*> arr_c_str;
    arr_c_str.push_back(data.c_str());
    std::map<std::string,int> attr;
    attr.insert(std::make_pair(StorageNode::TYPE, StorageNode::STRING));
    writeDataset1D(file, H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), name, arr_c_str.data(), attr, 1);
}
void cv::writeDataset(H5::H5File &file, const std::string &name, const cv::Mat &data)
{

    hsize_t fdims[2], mdims[1];
    fdims[0] = static_cast<hsize_t>(data.rows);
    fdims[1] = data.cols * data.elemSize();
    mdims[0] = fdims[1];

    H5::DataSpace fspace(2, fdims);
    H5::DataSpace mspace(1, mdims);

    H5::DataSet dataset(file.createDataSet(name.c_str(), H5::PredType::NATIVE_UCHAR, fspace));
    setIntAttribute(dataset, StorageNode::ROWS,  data.rows);
    setIntAttribute(dataset, StorageNode::COLS,  data.cols);
    setIntAttribute(dataset, StorageNode::MTYPE, data.type());
    setIntAttribute(dataset, StorageNode::TYPE,  StorageNode::USER);


    hsize_t offset[2], count[2], block[2];
    for (size_t i = 0; i < data.rows; i++)
    {
        offset[0] = i;
        offset[1] = 0;
        count[0]  = 1;
        count[1]  = 1;
        block[0]  = 1;
        block[1]  = fdims[1];
        fspace.selectHyperslab( H5S_SELECT_SET, count, offset, NULL, block);

        offset[0] = 0;
        count[0]  = mdims[0];
        block[0]  = 1;
        mspace.selectHyperslab( H5S_SELECT_SET, count, offset, NULL, block);

        dataset.write(data.ptr(i,0), H5::PredType::NATIVE_UCHAR, mspace, fspace);
    }
}
void cv::writeDataset(H5::H5File &file, const std::string &name, const SparseMat &data)
{
    Mat _tmp;
    data.copyTo(_tmp);
    writeDataset(file, name, _tmp);
}
void cv::writeDataset(H5::H5File &file, const std::string &name, const std::vector<int> &data)
{
    std::map<std::string,int> attr;
    attr.insert(std::make_pair(StorageNode::TYPE, StorageNode::SEQ));
    writeDataset1D(file, H5::PredType::NATIVE_INT, name, &data[0], attr, data.size());
}
void cv::writeDataset(H5::H5File &file, const std::string &name, const std::vector<float> &data)
{
    std::map<std::string,int> attr;
    attr.insert(std::make_pair(StorageNode::TYPE, StorageNode::SEQ));
    writeDataset1D(file, H5::PredType::NATIVE_FLOAT, name, &data[0], attr, data.size());
}
void cv::writeDataset(H5::H5File &file, const std::string &name, const std::vector<double> &data)
{
    std::map<std::string,int> attr;
    attr.insert(std::make_pair(StorageNode::TYPE, StorageNode::SEQ));
    writeDataset1D(file, H5::PredType::NATIVE_DOUBLE, name, &data[0], attr, data.size());
}
void cv::writeDataset(H5::H5File &file, const std::string &name, const std::vector<std::string> &vec)
{
    std::map<std::string,int> attr;
    attr.insert(std::make_pair(StorageNode::TYPE, StorageNode::SEQ));
    std::vector<const char*> arr_c_str;
    for (unsigned ii = 0; ii < vec.size(); ++ii)
        arr_c_str.push_back(vec[ii].c_str());
    writeDataset1D(file, H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), name, arr_c_str.data(), attr, vec.size());
}
void cv::writeDataset(H5::H5File &file, const std::string &name, const std::vector<cv::KeyPoint> &ks)
{
    struct type_opencv_kp
    {
        float x;
        float y;
        float size;
        float angle;
        float response;
        int octave;
        int class_id;
    };
    hsize_t fdims[1], mdims[1];
    fdims[0] = ks.size();
    mdims[0] = StorageNode::BUFFER; //buffer size of StorageNode::BUFFER keypoints

    H5::DataSpace fspace(1, fdims);
    H5::DataSpace mspace(1, mdims);

    H5::CompType mtype1(sizeof(type_opencv_kp));
    mtype1.insertMember( "x",       HOFFSET(type_opencv_kp, x),    H5::PredType::NATIVE_FLOAT);
    mtype1.insertMember( "y",       HOFFSET(type_opencv_kp, y),    H5::PredType::NATIVE_FLOAT);
    mtype1.insertMember( "size",    HOFFSET(type_opencv_kp, size), H5::PredType::NATIVE_FLOAT);
    mtype1.insertMember( "angle",   HOFFSET(type_opencv_kp, angle),    H5::PredType::NATIVE_FLOAT);
    mtype1.insertMember( "response",HOFFSET(type_opencv_kp, response), H5::PredType::NATIVE_FLOAT);
    mtype1.insertMember( "octave",  HOFFSET(type_opencv_kp, octave),   H5::PredType::NATIVE_INT);
    mtype1.insertMember( "class_i", HOFFSET(type_opencv_kp, class_id), H5::PredType::NATIVE_INT);
    H5::DataSet dataset(file.createDataSet(name, mtype1, fspace));
    setNodeType(dataset, StorageNode::SEQ);

    hsize_t offset[1], count[1], block[1];

    size_t step   = mdims[0];
    size_t chunks = (fdims[0] / mdims[0]) + ((fdims[0] % mdims[0])? 1 : 0);
    size_t extra  = (fdims[0] % mdims[0])? fdims[0] % mdims[0] : mdims[0];

    for (size_t i = 0, cursor = 0; i < chunks; i++, cursor+=step)
    {
        if (i == chunks - 1)
            step = extra;

        std::vector<type_opencv_kp> _kpts(step);
        for (size_t idx = 0; idx < step; idx++)
        {
            _kpts[idx].x = ks[cursor + idx].pt.x;
            _kpts[idx].y = ks[cursor + idx].pt.y;
            _kpts[idx].size = ks[cursor + idx].size;
            _kpts[idx].angle = ks[cursor + idx].angle;
            _kpts[idx].response = ks[cursor + idx].response;
            _kpts[idx].octave = ks[cursor + idx].octave;
            _kpts[idx].class_id = ks[cursor + idx].class_id;
        }

        offset[0] = cursor;
        count[0]  = step;
        block[0]  = 1;
        fspace.selectHyperslab( H5S_SELECT_SET, count, offset, NULL, block);
        offset[0] = 0;
        block[0]  = 1;
        mspace.selectHyperslab( H5S_SELECT_SET, count, offset, NULL, block);
        dataset.write(_kpts.data(), mtype1, mspace, fspace);
    }

}
void cv::writeDataset(H5::H5File &file, const std::string &name, const std::vector<cv::DMatch> &ks)
{
    struct type_opencv_dm
    {
        int queryIdx;
        int trainIdx;
        int imgIdx;
        float distance;
    };


    hsize_t fdims[1], mdims[1];
    fdims[0] = ks.size();
    mdims[0] = StorageNode::BUFFER; //buffer size of StorageNode::BUFFER keypoints

    H5::DataSpace fspace(1, fdims);
    H5::DataSpace mspace(1, mdims);

    H5::CompType mtype1(sizeof(type_opencv_dm));
    mtype1.insertMember( "queryIdx",  HOFFSET(type_opencv_dm, queryIdx),    H5::PredType::NATIVE_INT);
    mtype1.insertMember( "trainIdx",  HOFFSET(type_opencv_dm, trainIdx),    H5::PredType::NATIVE_INT);
    mtype1.insertMember( "imgIdx",    HOFFSET(type_opencv_dm, imgIdx),      H5::PredType::NATIVE_INT);
    mtype1.insertMember( "distance",  HOFFSET(type_opencv_dm, distance),    H5::PredType::NATIVE_FLOAT);
    H5::DataSet dataset(file.createDataSet(name, mtype1, fspace));
    setNodeType(dataset, StorageNode::SEQ);


    hsize_t offset[1], count[1], block[1];

    size_t step   = mdims[0];
    size_t chunks = (fdims[0] / mdims[0]) + ((fdims[0] % mdims[0])? 1 : 0);
    size_t extra  = (fdims[0] % mdims[0])? fdims[0] % mdims[0] : mdims[0];

    for (size_t i = 0, cursor = 0; i < chunks; i++, cursor+=step)
    {
        if (i == chunks - 1)
            step = extra;

        std::vector<type_opencv_dm> _kpts(step);
        for (size_t idx = 0; idx < step; idx++)
        {
            _kpts[idx].queryIdx = ks[cursor + idx].queryIdx;
            _kpts[idx].trainIdx = ks[cursor + idx].trainIdx;
            _kpts[idx].imgIdx   = ks[cursor + idx].imgIdx;
            _kpts[idx].distance = ks[cursor + idx].distance;
        }

        offset[0] = cursor;
        count[0]  = step;
        block[0]  = 1;
        fspace.selectHyperslab( H5S_SELECT_SET, count, offset, NULL, block);
        offset[0] = 0;
        block[0]  = 1;
        mspace.selectHyperslab( H5S_SELECT_SET, count, offset, NULL, block);
        dataset.write(_kpts.data(), mtype1, mspace, fspace);
    }

}

void cv::writeDataset(H5::H5File &file, const std::string &name, const Range& _data)
{
    std::vector<int> _tmp;
    _tmp.push_back(_data.start);
    _tmp.push_back(_data.end);
    writeDataset(file, name, _tmp);
}
//Read Basic Types
bool cv::readDataset(const H5::H5File &file, const std::string &name, int type, void *data)
{
    H5::DataSet dataset = file.openDataSet(name);
    int _type;
    bool read = getNodeType(dataset,  _type);
    if (read & (_type == type))
    {
        dataset.read(data, dataset.getDataType());
        return true;
    }
    return false;
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, int &data)
{
    return readDataset(file, name, StorageNode::INT, &data);
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, float &data)
{
    return readDataset(file, name, StorageNode::REAL, &data);
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, double &data)
{
    return readDataset(file, name, StorageNode::REAL, &data);
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, std::string &data)
{
    H5::DataSet dataset = file.openDataSet(name);
    int _type;
    bool read = getNodeType(dataset,  _type);
    if (read & (_type == StorageNode::STRING))
    {
        dataset.read(data, dataset.getDataType());
        return true;
    }
    return false;
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, cv::Mat &data)
{
    H5::DataSet dataset = file.openDataSet(name);
    int row, col, type, mtype;

    bool read = true;
    read &= getNodeType(dataset,  type);
    read &= getAttribute(dataset, H5::PredType::NATIVE_INT, StorageNode::ROWS,  &row);
    read &= getAttribute(dataset, H5::PredType::NATIVE_INT, StorageNode::COLS,  &col);
    read &= getAttribute(dataset, H5::PredType::NATIVE_INT, StorageNode::MTYPE, &mtype);
    read &= (type == StorageNode::USER);

    if (!read)
        return read;

    data.create(row, col, type);
    dataset.read(data.data, dataset.getDataType());
    return true;

}
bool cv::readDataset(const H5::H5File &file, const std::string &name, cv::SparseMat &data)
{
    //TODO Learn sparse mat memory structure.
    Mat _tmp;
    bool read = readDataset(file, name, _tmp);
    if (!read)
        return read;
    data = _tmp.clone();
    return true;
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, std::vector<cv::KeyPoint> &ks)
{
    struct type_opencv_kp
    {
        float x;
        float y;
        float size;
        float angle;
        float response;
        int octave;
        int class_id;
    };

    H5::DataSet dataset = file.openDataSet(name);
    H5::DataSpace fspace = dataset.getSpace();
    hsize_t fdims[1], mdims[1];
    int rank = fspace.getSimpleExtentDims( fdims, NULL);

    mdims[0] = StorageNode::BUFFER;
    H5::DataSpace mspace(rank, mdims);


    int type;
    bool read = getNodeType(dataset,  type) &
    (type == StorageNode::SEQ) &
    (rank == 1);

    if (!read)
        return read;


    hsize_t offset[1], count[1], block[1];

    size_t step   = mdims[0];
    size_t chunks = (fdims[0] / mdims[0]) + ((fdims[0] % mdims[0])? 1 : 0);
    size_t extra  = (fdims[0] % mdims[0])? fdims[0] % mdims[0] : mdims[0];

    for (size_t i = 0, cursor = 0; i < chunks; i++, cursor+=step)
    {
        if (i == chunks - 1)
            step = extra;

        std::vector<type_opencv_kp> _kpts(step);

        offset[0] = cursor;
        count[0]  = step;
        block[0]  = 1;
        fspace.selectHyperslab( H5S_SELECT_SET, count, offset, NULL, block);
        offset[0] = 0;
        block[0]  = 1;
        mspace.selectHyperslab( H5S_SELECT_SET, count, offset, NULL, block);
        dataset.read(_kpts.data(), dataset.getDataType(), mspace, fspace);


        for (size_t i = 0; i< step; i++)
        {
            cv::KeyPoint p;
            p.pt.x      = _kpts[i].x;
            p.pt.y      = _kpts[i].y;
            p.size      = _kpts[i].size;
            p.angle     = _kpts[i].angle;
            p.response  = _kpts[i].response;
            p.octave    = _kpts[i].octave;
            p.class_id  = _kpts[i].class_id;
            ks.push_back(p);
        }
    }
    return true;
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, std::vector<cv::DMatch> &dm)
{
    struct type_opencv_dm
    {
        int queryIdx;
        int trainIdx;
        int imgIdx;
        float distance;
    };

    H5::DataSet dataset = file.openDataSet(name);
    H5::DataSpace fspace = dataset.getSpace();
    hsize_t fdims[2], mdims[2];
    int rank = fspace.getSimpleExtentDims( fdims, NULL);

    mdims[0] = StorageNode::BUFFER;
    H5::DataSpace mspace(rank, mdims);

    int type;
    bool read = getNodeType(dataset,  type) &
    (type == StorageNode::SEQ) &
    (rank == 1);

    if (!read)
        return read;
    hsize_t offset[1], count[1], block[1];

    size_t step   = mdims[0];
    size_t chunks = (fdims[0] / mdims[0]) + ((fdims[0] % mdims[0])? 1 : 0);
    size_t extra  = (fdims[0] % mdims[0])? fdims[0] % mdims[0] : mdims[0];

    for (size_t i = 0, cursor = 0; i < chunks; i++, cursor+=step)
    {
        if (i == chunks - 1)
            step = extra;

        std::vector<type_opencv_dm> _dm(step);

        offset[0] = cursor;
        count[0]  = step;
        block[0]  = 1;
        fspace.selectHyperslab( H5S_SELECT_SET, count, offset, NULL, block);
        offset[0] = 0;
        block[0]  = 1;
        mspace.selectHyperslab( H5S_SELECT_SET, count, offset, NULL, block);
        dataset.read(_dm.data(), dataset.getDataType(), mspace, fspace);


        for (size_t i = 0; i< step; i++)
        {
            cv::DMatch p;
            p.queryIdx      = _dm[i].queryIdx;
            p.trainIdx      = _dm[i].trainIdx;
            p.imgIdx        = _dm[i].imgIdx;
            p.distance      = _dm[i].distance;
            dm.push_back(p);
        }
    }
    return true;
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, std::vector<int> &data)
{
    return readDataset1D<int>(file, name, data);
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, std::vector<float> &data)
{
    return readDataset1D<float>(file, name, data);
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, std::vector<double> &data)
{
    return readDataset1D<double>(file, name, data);
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, std::vector<std::string> &data)
{
    return readDataset1D<std::string>(file, name, data);
}
bool cv::readDataset(const H5::H5File &file, const std::string &name, cv::Range& _data)
{
    std::vector<int> _tmp;
    bool read = readDataset1D<int>(file, name, _tmp);
    if (!read)
        return read;
    _data.start = _tmp[0];
    _data.end = _tmp[1];

    return true;
}

bool cv::sortSequenceDatasets(const std::string &a, const std::string &b)
{
    size_t _ia, _ib;
    std::istringstream(a) >> _ia;
    std::istringstream(b) >> _ib;
    return _ia < _ib;
}
herr_t cv::readH5NodeInfo(hid_t loc_id, const char *name, const H5L_info_t *linfo, void *opdata)
{
    std::vector<std::string> *d = reinterpret_cast<std::vector<std::string> *>(opdata);
    d->push_back(name);
    return 0;
}

void cv::listSubnodes(H5::H5Location &node, std::vector<std::string> &subnodes)
{
    herr_t idx = H5Literate(node.getId(), H5_INDEX_NAME, H5_ITER_INC,
                            NULL, readH5NodeInfo, &subnodes);
    int type;
    bool read = getNodeType(node,type);
    if (read && type == StorageNode::SEQ)
    {
        std::sort(subnodes.begin(), subnodes.end(), cv::sortSequenceDatasets);
    }
}

