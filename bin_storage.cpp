#include "bin_storage.h"

cv::BFileStorage::BFileStorage(const string &filename, Mode flags): _mode(flags)
{
    if (flags == Mode::READ)
    {
        _fin.open(filename, ios::in | ios::binary);
        _status = _fin.is_open();
    }
    if (flags == Mode::WRITE)
    {
        _fout.open(filename, ios::out | ios::binary);
        _status = _fout.is_open();
    }
}
bool cv::BFileStorage::isOpened()
{
    return _status;
}
void cv::BFileStorage::release()
{
    if (!isOpened())
        return;
    if (_mode == Mode::READ)
        _fin.close();
    else if (_mode == Mode::WRITE)
        _fout.close();
}


