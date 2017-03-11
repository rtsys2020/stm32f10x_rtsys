#ifndef MifareUltralight_h
#define MifareUltralight_h

#include <PN532/PN532.h>
#include <NDEF/NfcTag.h>
#include <NDEF/Ndef.h>

class MifareUltralight
{
    public:
        MifareUltralight(PN532& nfcShield);
        ~MifareUltralight();
        NfcTag read(byte *uid, unsigned int uidLength);
    private:
        PN532* nfc;
        unsigned int tagCapacity;
        unsigned int messageLength;
        unsigned int bufferSize;
        unsigned int ndefStartIndex;
        boolean isUnformatted();
        void readCapabilityContainer();
        void findNdefMessage();
        void calculateBufferSize();
};

#endif
