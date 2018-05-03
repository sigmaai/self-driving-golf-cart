/****************************************************************************
** Meta object code from reading C++ file 'aerialmap_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../src/gps_viz/src/aerialmap_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'aerialmap_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz__AerialMapDisplay_t {
    QByteArrayData data[18];
    char stringdata0[254];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz__AerialMapDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz__AerialMapDisplay_t qt_meta_stringdata_rviz__AerialMapDisplay = {
    {
QT_MOC_LITERAL(0, 0, 22), // "rviz::AerialMapDisplay"
QT_MOC_LITERAL(1, 23, 19), // "updateDynamicReload"
QT_MOC_LITERAL(2, 43, 0), // ""
QT_MOC_LITERAL(3, 44, 11), // "updateAlpha"
QT_MOC_LITERAL(4, 56, 11), // "updateTopic"
QT_MOC_LITERAL(5, 68, 11), // "updateFrame"
QT_MOC_LITERAL(6, 80, 15), // "updateDrawUnder"
QT_MOC_LITERAL(7, 96, 15), // "updateObjectURI"
QT_MOC_LITERAL(8, 112, 10), // "updateZoom"
QT_MOC_LITERAL(9, 123, 12), // "updateBlocks"
QT_MOC_LITERAL(10, 136, 21), // "updateFrameConvention"
QT_MOC_LITERAL(11, 158, 16), // "initiatedRequest"
QT_MOC_LITERAL(12, 175, 15), // "QNetworkRequest"
QT_MOC_LITERAL(13, 191, 7), // "request"
QT_MOC_LITERAL(14, 199, 13), // "receivedImage"
QT_MOC_LITERAL(15, 213, 15), // "finishedLoading"
QT_MOC_LITERAL(16, 229, 12), // "errorOcurred"
QT_MOC_LITERAL(17, 242, 11) // "description"

    },
    "rviz::AerialMapDisplay\0updateDynamicReload\0"
    "\0updateAlpha\0updateTopic\0updateFrame\0"
    "updateDrawUnder\0updateObjectURI\0"
    "updateZoom\0updateBlocks\0updateFrameConvention\0"
    "initiatedRequest\0QNetworkRequest\0"
    "request\0receivedImage\0finishedLoading\0"
    "errorOcurred\0description"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz__AerialMapDisplay[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   79,    2, 0x09 /* Protected */,
       3,    0,   80,    2, 0x09 /* Protected */,
       4,    0,   81,    2, 0x09 /* Protected */,
       5,    0,   82,    2, 0x09 /* Protected */,
       6,    0,   83,    2, 0x09 /* Protected */,
       7,    0,   84,    2, 0x09 /* Protected */,
       8,    0,   85,    2, 0x09 /* Protected */,
       9,    0,   86,    2, 0x09 /* Protected */,
      10,    0,   87,    2, 0x09 /* Protected */,
      11,    1,   88,    2, 0x09 /* Protected */,
      14,    1,   91,    2, 0x09 /* Protected */,
      15,    0,   94,    2, 0x09 /* Protected */,
      16,    1,   95,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   17,

       0        // eod
};

void rviz::AerialMapDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        AerialMapDisplay *_t = static_cast<AerialMapDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateDynamicReload(); break;
        case 1: _t->updateAlpha(); break;
        case 2: _t->updateTopic(); break;
        case 3: _t->updateFrame(); break;
        case 4: _t->updateDrawUnder(); break;
        case 5: _t->updateObjectURI(); break;
        case 6: _t->updateZoom(); break;
        case 7: _t->updateBlocks(); break;
        case 8: _t->updateFrameConvention(); break;
        case 9: _t->initiatedRequest((*reinterpret_cast< QNetworkRequest(*)>(_a[1]))); break;
        case 10: _t->receivedImage((*reinterpret_cast< QNetworkRequest(*)>(_a[1]))); break;
        case 11: _t->finishedLoading(); break;
        case 12: _t->errorOcurred((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject rviz::AerialMapDisplay::staticMetaObject = {
    { &Display::staticMetaObject, qt_meta_stringdata_rviz__AerialMapDisplay.data,
      qt_meta_data_rviz__AerialMapDisplay,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz::AerialMapDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::AerialMapDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__AerialMapDisplay.stringdata0))
        return static_cast<void*>(const_cast< AerialMapDisplay*>(this));
    return Display::qt_metacast(_clname);
}

int rviz::AerialMapDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Display::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
