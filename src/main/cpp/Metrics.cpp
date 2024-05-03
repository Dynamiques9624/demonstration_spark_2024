#include <Metrics.h>
#include <chrono>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <fstream>
#include <iostream>
#include <inttypes.h>
#include <sys/stat.h>

using namespace std::chrono;
using namespace std;

// ----------------------------------------------------------------------------
//
Metrics::Metrics(const char *prefix, const char *basePath)
{
    this->basePath = basePath;
    this->prefix = prefix;
    this->filename = "";
    this->pathExist = false;
    this->lastFlush = this->now();
}

// ----------------------------------------------------------------------------
//
uint64_t Metrics::now() const
{
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

// ----------------------------------------------------------------------------
//
bool Metrics::mkdir(std::string &dirPath) const
{
    bool result = false;
    struct stat st;

    try
    {
        if (stat(dirPath.c_str(), &st) == 0)
        {
            if ((st.st_mode & S_IFMT) == S_IFDIR)
            {
                return true;
            }
        }

        string cmd = "mkdir -p " + dirPath;
        int status = system(cmd.c_str());

        if (status == -1)
        {
            cerr << "mkdir Error : " << strerror(errno) << endl;
        }
        else
        {
            result = true;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    return result;
}

// ----------------------------------------------------------------------------
//
bool Metrics::flush()
{
    bool result = false;

    try
    {
        if (!this->pathExist)
        {
            if (!this->mkdir(basePath))
            {
                std::cerr << "flush: mkdir failed" << '\n';
                return false;
            }
        }

        this->pathExist = true;

        uint64_t tm = now();
        char fname[512];

        snprintf(fname, 511, "%s/metrics-%s-%" PRIu64 ".json", basePath.c_str(), prefix.c_str(), tm);
        fname[511] = '\0';
        this->filename = fname;

        ofstream outfile;
        outfile.open(this->filename, ios::out | ios::trunc);

        outfile << "{" << endl;
        outfile << "\"metrics\":" << endl;
        outfile << "[" << endl;

        while (!queue.empty())
        {
            Data *d = queue.front();
            queue.pop_front();

            if (d != NULL)
            {
                toJSON(outfile, d);

                if (!queue.empty())
                    outfile << ",";

                outfile << endl;
                delete d;
            }
        }

        outfile << "]" << endl;
        outfile << "}" << endl;

        outfile.close();
        this->lastFlush = this->now();

        result = true;
    }
    catch (const exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    return result;
}

// ----------------------------------------------------------------------------
//
void Metrics::toJSON(ofstream &outfile, Data *data)
{
    try
    {
        string text = "";
        string escapedString = "";

        outfile << "  {";
        outfile << "\"timestamp\": ";
        outfile << data->timestamp_ms;
        outfile << ", ";

        outfile << "\"";
        outfile << data->name;
        outfile << "\": ";

        switch (data->type)
        {
        case VT_STRING:

            text = get<string>(data->value);
            escapeString(text, escapedString);

            outfile << "\"";
            outfile << escapedString;
            outfile << "\"";
            break;

        case VT_INT32:
            outfile << get<int32_t>(data->value);
            break;

        case VT_INT64:
            outfile << get<int64_t>(data->value);
            break;

        case VT_FLOAT:
            outfile << get<float>(data->value);
            break;

        case VT_DOUBLE:
            outfile << get<double>(data->value);
            break;

        case VT_BOOL:
            if (get<bool>(data->value))
                outfile << 1;
            else
                outfile << 0;
            break;

        default:
            outfile << 0;
            break;
        }

        outfile << "}";
    }
    catch (const exception &e)
    {
        cout << "Caught exception: '" << e.what() << "'\n";
    }
}

// ----------------------------------------------------------------------------
//
void Metrics::escapeString(string &text, string &result) const
{
    // Loop through each character in the input text
    for (size_t i = 0; i < text.length(); ++i)
    {
        if (text[i] == '\\' || text[i] == '"')
        {
            result.push_back('\\');
        }
        result.push_back(text[i]);
    }
}

// ----------------------------------------------------------------------------
//
void Metrics::publish(string name, string value)
{
    Data *rec = NULL;

    if (!this->data.count(name))
    {
        // New entry
        rec = new Data();
        rec->name = name;
        rec->type = VT_STRING;
        rec->timestamp_ms = this->now();
        rec->value = value;
        this->data[name] = rec;
        publish(rec);
    }
    else
    {
        rec = this->data[name];

        if (rec)
        {
            string _value = get<string>(rec->value);

            if (_value != value)
            {
                rec->timestamp_ms = this->now();
                rec->value = value;
                publish(rec);
            }
        }
    }
}

// ----------------------------------------------------------------------------
//
void Metrics::publish(string name, int32_t value)
{
    Data *rec = NULL;

    if (!this->data.count(name))
    {
        // New entry
        rec = new Data();
        rec->name = name;
        rec->type = VT_INT32;
        rec->timestamp_ms = this->now();
        rec->value = value;
        this->data[name] = rec;
        publish(rec);
    }
    else
    {
        rec = this->data[name];

        if (rec)
        {
            int32_t _value = get<int32_t>(rec->value);

            if (_value != value)
            {
                rec->timestamp_ms = this->now();
                rec->value = value;
                publish(rec);
            }
        }
    }
}

// ----------------------------------------------------------------------------
//
void Metrics::publish(string name, int64_t value)
{
    Data *rec = NULL;

    if (!this->data.count(name))
    {
        // New entry
        rec = new Data();
        rec->name = name;
        rec->type = VT_INT64;
        rec->timestamp_ms = this->now();
        rec->value = value;
        this->data[name] = rec;
        publish(rec);
    }
    else
    {
        rec = this->data[name];

        if (rec)
        {
            int64_t _value = get<int64_t>(rec->value);

            if (_value != value)
            {
                rec->timestamp_ms = this->now();
                rec->value = value;
                publish(rec);
            }
        }
    }
}

// ----------------------------------------------------------------------------
//
void Metrics::publish(string name, float value)
{
    Data *rec = NULL;

    if (!this->data.count(name))
    {
        // New entry
        rec = new Data();
        rec->name = name;
        rec->type = VT_FLOAT;
        rec->timestamp_ms = this->now();
        rec->value = value;
        this->data[name] = rec;
        publish(rec);
    }
    else
    {
        rec = this->data[name];

        if (rec)
        {
            float _value = get<float>(rec->value);

            if (_value != value)
            {
                rec->timestamp_ms = this->now();
                rec->value = value;
                publish(rec);
            }
        }
    }
}

// ----------------------------------------------------------------------------
//
void Metrics::publish(string name, double value)
{
    Data *rec = NULL;

    if (!this->data.count(name))
    {
        // New entry
        rec = new Data();
        rec->name = name;
        rec->type = VT_DOUBLE;
        rec->timestamp_ms = this->now();
        rec->value = value;
        this->data[name] = rec;
        publish(rec);
    }
    else
    {
        rec = this->data[name];

        if (rec)
        {
            double _value = get<double>(rec->value);

            if (_value != value)
            {
                rec->timestamp_ms = this->now();
                rec->value = value;
                publish(rec);
            }
        }
    }
}

// ----------------------------------------------------------------------------
//
void Metrics::publish(string name, bool value)
{
    Data *rec = NULL;

    if (!this->data.count(name))
    {
        // New entry
        rec = new Data();
        rec->name = name;
        rec->type = VT_BOOL;
        rec->timestamp_ms = this->now();
        rec->value = value;
        this->data[name] = rec;
        publish(rec);
    }
    else
    {
        rec = this->data[name];

        if (rec)
        {
            bool _value = get<bool>(rec->value);

            if (_value != value)
            {
                auto t = this->now();
                auto delay = t - rec->timestamp_ms;

                rec->timestamp_ms = t;
                rec->value = value;
                
                if (delay >= MIN_PUBLISH_INTERVAL_MS)
                    publish(rec);
            }
        }
    }
}

// ----------------------------------------------------------------------------
//
void Metrics::publish(Data *data)
{
    queue.push_back(clone(data));

    uint64_t delay = this->now() - this->lastFlush;

    if (queue.size() >= MAX_QUEUE_SIZE || delay >= MAX_MAX_FLUSH_INTERVAL)
    {
        flush();
    }
}

// ----------------------------------------------------------------------------
//
Metrics::Data *Metrics::clone(Metrics::Data *data)
{
    try
    {
        Data *dst = new Data();

        dst->timestamp_ms = data->timestamp_ms;
        dst->name = data->name;
        dst->type = data->type;
        dst->value = data->value;

        return dst;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return NULL;
    }
}
