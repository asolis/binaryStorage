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

#ifndef __BIN__STORAGE__H
#define __BIN__STORAGE__H

#include <map>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

template<class T> inline void write(ostream& out, const T& data) {
	out.write(reinterpret_cast<const char*>(&data), sizeof(T));
}

template<class T> void write(ostream& out, const vector<T>& vec) {
	write(out, vec.size());
    for (typename  vector<T>::const_iterator it = vec.begin(); it!=vec.end(); it++)
        write(out, *it);
}
template<class K, class V> void write(ostream& out, const map<K,V> &dict)
{
    write(out, dict.size());
    for (typename map<K,V>::const_iterator it=dict.begin(); it!=dict.end(); ++it)
    {
        write(out, it->first);
        write(out, it->second);
    }
}
template<class T> void write(ostream& out, const vector<vector<T>> &vec)
{
    write(out, vec.size());
    for (typename vector<vector<T>>::const_iterator it = vec.begin(); it!=vec.end(); it++)
        write(out, *it);
}
template<> void write<Mat>(ostream& out, const Mat &_data);

template<class T> inline void read(istream& in, T& data) {
    in.read(reinterpret_cast<char*>(&data), sizeof(T));
}
template<class T> void read(istream& in, vector<T>& vec) {
	size_t size;
	read(in, size);
	vec.resize(size);
    for (typename vector<T>::iterator it; it != vec.end(); it++)
        read(in, *it);
}
template<class K, class V> void read(istream& in,map<K,V> &dict)
{
    size_t size;
    read(in, size);
    for (size_t i = 0; i < size; i++)
    {
        K key;
        V value;
        read(in, key);
        read(in, value);
        dict.insert(pair<K,V>(key,value));
    }
}
template<class T> void read(istream& in, vector<vector<T>> &vec)
{
    size_t size;
    read(in, size);
    vec.resize(size);
    for (typename vector<vector<T>>::iterator it; it != vec.end(); it++)
        read(in, *it);
}
template<> void read<Mat>(istream& in, Mat &_data);


#endif 
