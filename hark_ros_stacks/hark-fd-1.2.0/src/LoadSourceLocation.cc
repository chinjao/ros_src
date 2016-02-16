/*
 * Copyright 2008 Kyoto University and Honda Motor Co.,Ltd.
 * All rights reserved.
 * HARK was developed by researchers in Okuno Laboratory
 * at the Kyoto University and Honda Research Institute Japan Co.,Ltd.
 */

#include <iostream>
#include "BufferedNode.h"
#include "Buffer.h"
#include "Vector.h"
#include <math.h>
#include "Source.h"

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;
using namespace FD;

class LoadSourceLocation;

DECLARE_NODE(LoadSourceLocation);
/*Node
 *
 * @name LoadSourceLocation
 * @category HARK:Localization
 * @description Load source locations from a file with the "Source" format.
 *
 * @output_name SOURCES
 * @output_type Vector<ObjectRef>
 * @output_description Source locations with IDs. Each element of the vector is a source location with ID specified by "Source".
 *
 * @output_name NOT_EOF
 * @output_type bool
 * @output_description True if the current frame does not reach the End-of-File.
 *
 * @parameter_name FILENAME
 * @parameter_type string
 * @parameter_description The filename to load source locations.
 *
 END*/

class LoadSourceLocation : public BufferedNode {
  int sourcesID;
  int noteofID;

  string filename;

  ifstream fin;
  int read_count;

public:
  LoadSourceLocation(string nodeName, ParameterSet params)
    : BufferedNode(nodeName, params)
  {
    sourcesID = addOutput("SOURCES");
    noteofID = addOutput("NOT_EOF");

    filename = object_cast<string>(parameters.get("FILENAME"));
    if (filename.c_str() == '\0') {
      throw new NodeException(NULL, string("FILENAME is empty."), __FILE__, __LINE__);
    }

    fin.open(filename.c_str());
    if(fin.fail())
      {
        throw new NodeException(NULL, string("Can't open '")+filename+string("'."), __FILE__, __LINE__);
      }

    inOrder = true;
  }

  void calculate(int output_id, int count, Buffer &out)
  {
    Vector<ObjectRef>& sources = *new Vector<ObjectRef>;
    (*(outputs[sourcesID].buffer))[count] = &sources;
    string buffer;
    vector<string> tokens;

    getline(fin, buffer);
    (*(outputs[noteofID].buffer))[count] = (fin.eof() ? FalseObject : TrueObject);

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer_space;
    boost::char_separator<char> sep(" ");
    tokenizer_space tokenizer(buffer, sep);
    for (tokenizer_space::iterator it = tokenizer.begin();
         it != tokenizer.end(); ++it) {
      tokens.push_back(*it);
    }
    int source_count = 0;
    if ((tokens.size() - 1) % 4 == 0) {
      source_count = (tokens.size() - 1) / 4;
      if (lexical_cast<int>(tokens[0]) != count) {
        throw new NodeException(this, "The input of source location data is not correct.",
                                __FILE__, __LINE__);
      }
    }

    for (int i = 0; i < source_count; i++) {
      RCPtr<Source> src(new Source);

      src->power = 100.0f;
      src->id = lexical_cast<int>(tokens[i * 4 + 1]);
      src->x[0] = lexical_cast<float>(tokens[i * 4 + 2]);
      src->x[1] = lexical_cast<float>(tokens[i * 4 + 3]);
      src->x[2] = lexical_cast<float>(tokens[i * 4 + 4]);

      sources.push_back(src);
    }

  }
};
