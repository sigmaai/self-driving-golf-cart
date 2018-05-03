/****************************************************************************
** Meta object code from reading C++ file 'tileloader.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/gps_viz/src/tileloader.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tileloader.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_TileLoader_t {
    QByteArrayData data[12];
    char stringdata0[145];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TileLoader_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TileLoader_t qt_meta_stringdata_TileLoader = {
    {
QT_MOC_LITERAL(0, 0, 10), // "TileLoader"
QT_MOC_LITERAL(1, 11, 16), // "initiatedRequest"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 15), // "QNetworkRequest"
QT_MOC_LITERAL(4, 45, 7), // "request"
QT_MOC_LITERAL(5, 53, 13), // "receivedImage"
QT_MOC_LITERAL(6, 67, 15), // "finishedLoading"
QT_MOC_LITERAL(7, 83, 12), // "errorOcurred"
QT_MOC_LITERAL(8, 96, 11), // "description"
QT_MOC_LITERAL(9, 108, 15), // "finishedRequest"
QT_MOC_LITERAL(10, 124, 14), // "QNetworkReply*"
QT_MOC_LITERAL(11, 139, 5) // "reply"

    },
    "TileLoader\0initiatedRequest\0\0"
    "QNetworkRequest\0request\0receivedImage\0"
    "finishedLoading\0errorOcurred\0description\0"
    "finishedRequest\0QNetworkReply*\0reply"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TileLoader[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       5,    1,   42,    2, 0x06 /* Public */,
       6,    0,   45,    2, 0x06 /* Public */,
       7,    1,   46,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    1,   49,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    8,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 10,   11,

       0        // eod
};

void TileLoader::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TileLoader *_t = static_cast<TileLoader *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->initiatedRequest((*reinterpret_cast< QNetworkRequest(*)>(_a[1]))); break;
        case 1: _t->receivedImage((*reinterpret_cast< QNetworkRequest(*)>(_a[1]))); break;
        case 2: _t->finishedLoading(); break;
        case 3: _t->errorOcurred((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->finishedRequest((*reinterpret_cast< QNetworkReply*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (TileLoader::*_t)(QNetworkRequest );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TileLoader::initiatedRequest)) {
                *result = 0;
            }
        }
        {
            typedef void (TileLoader::*_t)(QNetworkRequest );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TileLoader::receivedImage)) {
                *result = 1;
            }
        }
        {
            typedef void (TileLoader::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TileLoader::finishedLoading)) {
                *result = 2;
            }
        }
        {
            typedef void (TileLoader::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TileLoader::errorOcurred)) {
                *result = 3;
            }
        }
    }
}

const QMetaObject TileLoader::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_TileLoader.data,
      qt_meta_data_TileLoader,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *TileLoader::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TileLoader::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_TileLoader.stringdata0))
        return static_cast<void*>(const_cast< TileLoader*>(this));
    return QObject::qt_metacast(_clname);
}

int TileLoader::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void TileLoader::initiatedRequest(QNetworkRequest _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void TileLoader::receivedImage(QNetworkRequest _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void TileLoader::finishedLoading()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void TileLoader::errorOcurred(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
