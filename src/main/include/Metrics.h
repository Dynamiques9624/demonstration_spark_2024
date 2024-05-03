#pragma once

#include <string>
#include <map>
#include <list>
#include <variant>

#define METRICS_BASE_PATH "/home/lvuser/data/metrics"
#define METRICS_PREFIX "metrics_"
#define MAX_QUEUE_SIZE 5000
#define MAX_MAX_FLUSH_INTERVAL (60 * 1000)
#define MIN_PUBLISH_INTERVAL_MS 250

// ----------------------------------------------------------------------------
//
class Metrics
{
public:
    Metrics(const char *prefix = METRICS_PREFIX, const char *basePath = METRICS_BASE_PATH);

    uint64_t now() const;
    void escapeString(std::string &text, std::string &result) const;
    bool mkdir(std::string &dirPath) const;

    bool flush();
    void publish(std::string name, std::string value);
    void publish(std::string name, int32_t value);
    void publish(std::string name, int64_t value);
    void publish(std::string name, float value);
    void publish(std::string name, double value);
    void publish(std::string name, bool value);

protected:
private:
    enum ValueType
    {
        VT_STRING,
        VT_INT32,
        VT_INT64,
        VT_FLOAT,
        VT_DOUBLE,
        VT_BOOL
    };

    struct Data
    {
        uint64_t timestamp_ms;
        std::string name;
        ValueType type;
        std::variant<std::string, int32_t, int64_t, float, double, bool> value;
    };

    std::map<std::string, Data*> data;
    std::list<Data *> queue;
    std::string basePath;
    std::string prefix;
    std::string filename;
    bool pathExist;
    uint64_t lastFlush;

    void publish(Data *_data);
    void toJSON(std::ofstream &outfile, Data *data);
    Metrics::Data* clone(Metrics::Data *data);
};
