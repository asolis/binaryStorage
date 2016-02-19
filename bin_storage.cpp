#include "bin_storage.h"


template<> void write<Mat>(ostream& out,const Mat &_data)
{
        write(out, _data.rows);
        write(out, _data.cols);
        write(out, _data.type());
        const size_t bytes = _data.cols * _data.elemSize();
        for (int i = 0; i < _data.rows; i++)
            out.write(reinterpret_cast<const char*>(_data.ptr(i, 0)), bytes);
}

template<> void read<Mat>(istream& in,Mat &_data)
{
    int rows, cols, type;
    read(in, rows);
    read(in, cols);
    read(in, type);
    _data.create(rows, cols, type);
    const size_t bytes = _data.cols * _data.elemSize();
    for (int i = 0; i < _data.rows; i++)
        in.read(reinterpret_cast<char*>(_data.ptr(i, 0)), bytes);
}
