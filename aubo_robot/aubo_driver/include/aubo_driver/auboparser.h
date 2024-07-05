#ifndef AUBOPARSER_H
#define AUBOPARSER_H
#include "gparser.h"
#include <vector>
//class GParser;
class AuboParser : public GParser
{
public:
    AuboParser();

    static AuboParser *instance();

    void ParsingFile(const string& file_name,list<paramPoint>& List);
private:
    std::vector<string> split(const string& str, const string& delim);

    void addNullNode(list<paramPoint>& List);

    string& trim(string &);

};

#endif // AUBOPARSER_H
