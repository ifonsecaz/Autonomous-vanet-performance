//
// Generated file, do not edit! Created by nedtool 5.7 from veins_inet/VeinsInetSampleMessage.msg.
//

#ifndef __VEINSINETSAMPLEMESSAGE_M_H
#define __VEINSINETSAMPLEMESSAGE_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0507
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif

// dll export symbol
#ifndef VEINS_INET_API
#  if defined(VEINS_INET_EXPORT)
#    define VEINS_INET_API  OPP_DLLEXPORT
#  elif defined(VEINS_INET_IMPORT)
#    define VEINS_INET_API  OPP_DLLIMPORT
#  else
#    define VEINS_INET_API
#  endif
#endif



class VeinsInetSampleMessage;
#include "inet/common/INETDefs_m.h" // import inet.common.INETDefs

#include "inet/common/packet/chunk/Chunk_m.h" // import inet.common.packet.chunk.Chunk

/**
 * Class generated from <tt>veins_inet/VeinsInetSampleMessage.msg:34</tt> by nedtool.
 * <pre>
 * //
 * // Example message definition
 * //
 * class VeinsInetSampleMessage extends inet::FieldsChunk
 * {
 *     string roadId;
 * }
 * </pre>
 */
class VEINS_INET_API VeinsInetSampleMessage : public ::inet::FieldsChunk
{
  protected:
    omnetpp::opp_string roadId;

  private:
    void copy(const VeinsInetSampleMessage& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const VeinsInetSampleMessage&);

  public:
    const char* msgr[9];

    int msgRec;
    double lastC;

    int env0;
    int env1;
    int env2;
    int env3;
    int env4;
    int env5;
    int env6;
    int env7;
    int env8;

    int renv0;
    int renv1;
    int renv2;
    int renv3;
    int renv4;
    int renv5;
    int renv6;
    int renv7;
    int renv8;

    const char* pastVal0;
    const char* pastVal1;
    const char* pastVal2;
    const char* pastVal3;
    const char* pastVal4;
    const char* pastVal5;
    const char* pastVal6;
    const char* pastVal7;
    const char* pastVal8;
    const char* pastVal01;
    const char* pastVal11;
    const char* pastVal21;
    const char* pastVal31;
    const char* pastVal41;
    const char* pastVal51;
    const char* pastVal61;
    const char* pastVal71;
    const char* pastVal81;
    const char* pastVal02;
    const char* pastVal12;
    const char* pastVal22;
    const char* pastVal32;
    const char* pastVal42;
    const char* pastVal52;
    const char* pastVal62;
    const char* pastVal72;
    const char* pastVal82;
    const char* pastVal03;
        const char* pastVal13;
        const char* pastVal23;
        const char* pastVal33;
        const char* pastVal43;
        const char* pastVal53;
        const char* pastVal63;
        const char* pastVal73;
        const char* pastVal83;
    const char* pastVal9;
    const char* pastVal04;
        const char* pastVal14;
        const char* pastVal24;
        const char* pastVal34;
        const char* pastVal44;
        const char* pastVal54;
        const char* pastVal64;
        const char* pastVal74;
        const char* pastVal84;
        const char* pastVal05;
            const char* pastVal15;
            const char* pastVal25;
            const char* pastVal35;
            const char* pastVal45;
            const char* pastVal55;
            const char* pastVal65;
            const char* pastVal75;
            const char* pastVal85;

    VeinsInetSampleMessage();
    VeinsInetSampleMessage(const VeinsInetSampleMessage& other);
    virtual ~VeinsInetSampleMessage();
    VeinsInetSampleMessage& operator=(const VeinsInetSampleMessage& other);
    virtual VeinsInetSampleMessage *dup() const override {return new VeinsInetSampleMessage(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual const char * getRoadId() const;
    virtual void setRoadId(const char * roadId);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const VeinsInetSampleMessage& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, VeinsInetSampleMessage& obj) {obj.parsimUnpack(b);}

#endif // ifndef __VEINSINETSAMPLEMESSAGE_M_H

