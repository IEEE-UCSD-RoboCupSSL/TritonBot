#ifndef __BOOST_LOGGER_H
#define __BOOST_LOGGER_H

#include "boost/log/utility/formatting_ostream.hpp"
#include <boost/log/sinks/async_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/core/null_deleter.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <ostream>





#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/log/attributes/scoped_attribute.hpp>

#include <iostream>
#include <boost/chrono.hpp>
#include <boost/chrono/system_clocks.hpp>
#include <boost/thread.hpp> 


enum SeverityLevel
{
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Fatal
};


#define LOG_TAG(str) "Tag", boost::log::attributes::constant< std::string >(str)
BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", SeverityLevel);
BOOST_LOG_ATTRIBUTE_KEYWORD(line_id, "LineID", unsigned int)
BOOST_LOG_ATTRIBUTE_KEYWORD(tag_attr, "Tag", std::string);
BOOST_LOG_ATTRIBUTE_KEYWORD(timeline, "Timeline", boost::log::attributes::timer::value_type)





std::ostream& operator<< (std::ostream& strm, SeverityLevel level);


class BLogger {
private:
    typedef boost::shared_ptr<boost::log::sources::severity_logger<SeverityLevel>> slog_ptr;
    typedef boost::log::sinks::asynchronous_sink< boost::log::sinks::text_ostream_backend > text_sink;
    

public:
    SeverityLevel sev = Info;
    slog_ptr slog;
    static boost::shared_ptr< text_sink > sink; 
    static bool statically_init;

    // should be called before the construction
    static void staticInit(); 

    // default constructor: logging to std::clog
    BLogger();

    void add_tag(std::string tag);
    
    static void setToShorterFormat();
    static void set_even_shorter_format();
    static void set_shortest_format();
    static void set_default_format();

    BLogger& operator()(SeverityLevel sev);

    void log(SeverityLevel sev, std::string str);

};

BLogger& operator<<(BLogger& logger, std::string& str);
BLogger& operator<<(BLogger& logger, const char* str);

#endif