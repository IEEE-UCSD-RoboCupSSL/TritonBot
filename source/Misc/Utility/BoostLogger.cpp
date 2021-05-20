#include "Misc/Utility/BoostLogger.hpp"


namespace expr = boost::log::expressions;


typedef boost::log::sinks::asynchronous_sink< boost::log::sinks::text_ostream_backend > text_sink;

bool BLogger::statically_init = false;
boost::shared_ptr< text_sink > BLogger::sink = boost::make_shared< text_sink >();

void BLogger::staticInit() {

    
    // set where to log to
    sink->locked_backend()->add_stream(boost::shared_ptr<std::ostream>(&std::cout, boost::null_deleter()));
    // set default format
    set_default_format();

    // default filter level
    sink->set_filter(severity >= Info);

    boost::log::add_common_attributes();

    boost::log::core::get()->add_global_attribute("Timeline", boost::log::attributes::timer());
    boost::log::core::get()->add_global_attribute("ThreadID", boost::log::attributes::current_thread_id());
    boost::log::core::get()->add_sink(sink);

    statically_init = true;
}






// default constructor: logging to std::clog with default format
BLogger::BLogger() {

    if(BLogger::statically_init != true) { 
        BOOST_LOG_TRIVIAL(error) << "\033[0;31mlogger static initializer has not yet been called\033[0m";
    }
    // construct the logger
    slog = slog_ptr(new boost::log::sources::severity_logger<SeverityLevel>);

    
}
    

void BLogger::setToShorterFormat() {
    sink->set_formatter
    (
        expr::stream
            << "<" << severity << "> "
            << expr::if_(expr::has_attr(tag_attr))
               [
                    expr::stream << "[" << tag_attr << "] "
               ]
            << expr::smessage
    );

}

void BLogger::set_shortest_format() {
    sink->set_formatter
    (
        expr::stream
            << expr::smessage
    );
}

void BLogger::set_even_shorter_format() {
    sink->set_formatter
    (
        expr::stream
            << expr::if_(expr::has_attr(tag_attr))
               [
                    expr::stream << "[" << tag_attr << "] "
               ]
            << expr::smessage
    );
}

void BLogger::set_default_format() {
        // set default format
    sink->set_formatter
    (
        expr::stream
            << std::dec << std::setw(6) << std::setfill('0') 
                << expr::attr< unsigned int >("LineID") 
                << std::dec << std::setfill(' ') << " "
            << "[ThreadID: " 
                << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID")<< "]"
                
            << expr::if_(expr::has_attr(timeline))
               [
                    expr::stream << "[Time: " << timeline << "] "
               ]
            << ": <" << severity << "> "
            << expr::if_(expr::has_attr(tag_attr))
               [
                    expr::stream << "[" << tag_attr << "] "
               ]
            << expr::smessage
            
    );
}

void BLogger::add_tag(std::string tag) {
    slog->add_attribute(LOG_TAG(tag));
}

BLogger& BLogger::operator()(SeverityLevel sev) {
    this->sev = sev;
    return *this;
}


BLogger& operator<<(BLogger& logger, std::string str)
{
    BOOST_LOG_SEV(*(logger.slog), logger.sev) << str; 
    logger.sink.get()->flush();
    return logger;
}

BLogger& operator<<(BLogger& logger, const char* str) {
    BOOST_LOG_SEV(*(logger.slog), logger.sev) << std::string(str);
    logger.sink.get()->flush();
    return logger;
}


std::ostream& operator<< (std::ostream& strm, SeverityLevel level)
{
    static const char* strings[] =
    {
        "trace",
        "debug",
        "info",
        "warning",
        "error",
        "fatal"
    };

    if (static_cast< std::size_t >(level) < sizeof(strings) / sizeof(*strings))
        strm << strings[level];
    else
        strm << static_cast< int >(level);

    strm.flush();

    return strm;
}




void BLogger::log(SeverityLevel sev, std::string str) {
    BOOST_LOG_SEV(*(this->slog), sev) << std::string(str);
    sink.get()->flush();
}