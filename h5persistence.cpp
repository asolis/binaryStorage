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
#include "h5persistence.hpp"

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




