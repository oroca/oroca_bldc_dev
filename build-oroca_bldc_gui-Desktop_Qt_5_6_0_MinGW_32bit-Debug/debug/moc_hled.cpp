/****************************************************************************
** Meta object code from reading C++ file 'hled.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.6.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../oroca_bldc_gui/hled.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'hled.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.6.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_HLed_t {
    QByteArrayData data[10];
    char stringdata0[60];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_HLed_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_HLed_t qt_meta_stringdata_HLed = {
    {
QT_MOC_LITERAL(0, 0, 4), // "HLed"
QT_MOC_LITERAL(1, 5, 8), // "setColor"
QT_MOC_LITERAL(2, 14, 0), // ""
QT_MOC_LITERAL(3, 15, 5), // "color"
QT_MOC_LITERAL(4, 21, 6), // "toggle"
QT_MOC_LITERAL(5, 28, 6), // "turnOn"
QT_MOC_LITERAL(6, 35, 2), // "on"
QT_MOC_LITERAL(7, 38, 7), // "turnOff"
QT_MOC_LITERAL(8, 46, 3), // "off"
QT_MOC_LITERAL(9, 50, 9) // "ledStatus"

    },
    "HLed\0setColor\0\0color\0toggle\0turnOn\0"
    "on\0turnOff\0off\0ledStatus"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_HLed[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x0a /* Public */,
       4,    0,   52,    2, 0x0a /* Public */,
       5,    1,   53,    2, 0x0a /* Public */,
       5,    0,   56,    2, 0x2a /* Public | MethodCloned */,
       7,    1,   57,    2, 0x0a /* Public */,
       7,    0,   60,    2, 0x2a /* Public | MethodCloned */,
       9,    0,   61,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QColor,    3,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    8,
    QMetaType::Void,
    QMetaType::Bool,

       0        // eod
};

void HLed::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        HLed *_t = static_cast<HLed *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setColor((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 1: _t->toggle(); break;
        case 2: _t->turnOn((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->turnOn(); break;
        case 4: _t->turnOff((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->turnOff(); break;
        case 6: { bool _r = _t->ledStatus();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObject HLed::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_HLed.data,
      qt_meta_data_HLed,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *HLed::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *HLed::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_HLed.stringdata0))
        return static_cast<void*>(const_cast< HLed*>(this));
    return QWidget::qt_metacast(_clname);
}

int HLed::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
