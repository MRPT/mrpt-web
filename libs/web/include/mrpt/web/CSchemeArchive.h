#pragma once
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <string_view>
#include <string>
#include <memory>

namespace mrpt::web
{
template <typename SCHEME_CAPABLE>
class CSchemeArchive : public mrpt::serialization::CSchemeArchiveBase_impl
{
  public:
    CSchemeArchive(SCHEME_CAPABLE& val):m_val(val) {}
    //Virtual assignment operators
    virtual mrpt::serialization::CSchemeArchiveBase &operator=(const int32_t val) override
    {
        m_val = val;
        return *m_parent;
    }
    virtual mrpt::serialization::CSchemeArchiveBase &operator=(const uint32_t val) override
    {
        m_val = val;
        return *m_parent;
    }
    virtual mrpt::serialization::CSchemeArchiveBase &operator=(const int64_t val) override
    {
        m_val = val;
        return *m_parent;
    }
    virtual mrpt::serialization::CSchemeArchiveBase &operator=(const uint64_t val) override
    {
        m_val = val;
        return *m_parent;
    }
    virtual mrpt::serialization::CSchemeArchiveBase &operator=(const float val) override
    {
        m_val = val;
        return *m_parent;
    }
    virtual mrpt::serialization::CSchemeArchiveBase &operator=(const double val) override
    {
        m_val = val;
        return *m_parent;
    }
    virtual mrpt::serialization::CSchemeArchiveBase &operator=(const std::nullptr_t val) override
    {
        m_val = val;
        return *m_parent;
    }
    virtual mrpt::serialization::CSchemeArchiveBase &operator=(const std::string val) override
    {
        m_val = val;
        return *m_parent;
    }
    virtual mrpt::serialization::CSchemeArchiveBase &operator=(const bool val) override
    {
        m_val = val;
        return *m_parent;
    }

    virtual explicit operator int32_t() const override
    {
        return m_val.asInt();
    }
    virtual explicit operator uint32_t() const override
    {
        return m_val.asUInt();
    }
    virtual explicit operator int64_t() const override
    {
        return m_val.asInt64();
    }
    virtual explicit operator uint64_t() const override
    {
        return m_val.asUInt64();
    }
    virtual explicit operator float() const override
    {
        return m_val.asFloat();
    }
    virtual explicit operator double() const override
    {
        return m_val.asDouble();
    }
    virtual explicit operator bool() const override
    {
        return m_val.asBool();
    }
    virtual explicit operator std::string() const override
    {
        return m_val.asString();
    }

    virtual mrpt::serialization::CSchemeArchiveBase &operator=(mrpt::serialization::CSerializable& obj) override
    {
        ReadObject(*m_parent, obj);
        return *m_parent;
    }
    virtual void asSerializableObject(mrpt::serialization::CSerializable& obj) override
    {
        WriteObject(*m_parent, obj);
        return;
    }

    virtual mrpt::serialization::CSchemeArchiveBase operator[](size_t idx) override
    {
        return mrpt::serialization::CSchemeArchiveBase(std::make_unique<CSchemeArchive<SCHEME_CAPABLE>>(m_val[(int)idx]));
    }
    virtual mrpt::serialization::CSchemeArchiveBase operator[](std::string str) override
    {
        return mrpt::serialization::CSchemeArchiveBase(std::make_unique<CSchemeArchive<SCHEME_CAPABLE>>(m_val[std::string(str)]));
    }

    private:
        SCHEME_CAPABLE& m_val;
};

}
