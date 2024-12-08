#ifndef SERIALCONVERTER_H
#define SERIALCONVERTER_H

#include <vector>
#include <Arduino.h>

using Bytes = std::vector<uint8_t>;

class SerialConverter {
public:
    // Direct Bytes to Bytes conversion for ASCII HEX
    static inline Bytes bytesToHex(const Bytes& input) {
        Bytes result;
        result.reserve(input.size() * 3);  // Chaque octet devient 2 caractères + 1 espace
        
        for(uint8_t byte : input) {
            // Premier caractère (high nibble)
            result.push_back(nibbleToHexChar(byte >> 4));
            // Second caractère (low nibble)
            result.push_back(nibbleToHexChar(byte & 0x0F));
            // Optionnel: ajouter un espace entre les octets
            result.push_back(' ');
        }
        
        return result;
    }
    
    static inline Bytes hexToBytes(const Bytes& input) {
        Bytes result;
        Bytes cleaned;
        cleaned.reserve(input.size());
        
        // Première passe : on nettoie et normalise l'entrée
        // - Supprime les espaces
        // - Ignore 0x ou 0X en début de chaîne
        // - Convertit tout en minuscule
        size_t start = (input.size() > 2 && input[0] == '0' && (input[1] == 'x' || input[1] == 'X')) ? 2 : 0;
        
        for(size_t i = start; i < input.size(); i++) {
            uint8_t c = input[i];
            if(!isSpace(c)) {  // Ignore les espaces
                if(isxdigit(c)) {  // Ne garde que les caractères hexa valides
                    cleaned.push_back(tolower(c));
                }
            }
        }
        
        // Si nombre impair de caractères, on ajoute un 0 devant
        if(cleaned.size() % 2) {
            cleaned.insert(cleaned.begin(), '0');
        }
        
        // Réserve la taille exacte du résultat
        result.reserve(cleaned.size() / 2);
        
        // Deuxième passe : conversion des paires de caractères en octets
        for(size_t i = 0; i < cleaned.size(); i += 2) {
            uint8_t high = hexCharToNibble(cleaned[i]);
            uint8_t low = hexCharToNibble(cleaned[i + 1]);
            if(high != 0xFF && low != 0xFF) {  // Vérification de validité
                result.push_back((high << 4) | low);
            }
        }
        
        return result;
    }
    
    // Direct Bytes to Bytes conversion for Base64
    static inline Bytes bytesToBase64(const Bytes& input) {
        Bytes result;
        result.reserve((input.size() * 4 + 2) / 3);  // Taille approximative
        
        for(size_t i = 0; i < input.size(); i += 3) {
            uint32_t n = input[i] << 16;
            if(i + 1 < input.size()) n |= input[i + 1] << 8;
            if(i + 2 < input.size()) n |= input[i + 2];
            
            result.push_back(base64Char((n >> 18) & 0x3F));
            result.push_back(base64Char((n >> 12) & 0x3F));
            
            if(i + 1 < input.size())
                result.push_back(base64Char((n >> 6) & 0x3F));
            else
                result.push_back('=');
                
            if(i + 2 < input.size())
                result.push_back(base64Char(n & 0x3F));
            else
                result.push_back('=');
        }
        
        return result;
    }
    
    static inline Bytes base64ToBytes(const Bytes& input) {
        Bytes result;
        size_t padding = 0;
        
        // Compte le padding
        for(auto it = input.rbegin(); it != input.rend() && *it == '='; ++it) {
            padding++;
        }
        
        result.reserve((input.size() * 3) / 4);
        
        for(size_t i = 0; i < input.size() - padding; i += 4) {
            uint32_t n = base64Value(input[i]) << 18;
            n += base64Value(input[i + 1]) << 12;
            
            result.push_back((n >> 16) & 0xFF);
            
            if(i + 2 < input.size() - padding) {
                n += base64Value(input[i + 2]) << 6;
                result.push_back((n >> 8) & 0xFF);
            }
            
            if(i + 3 < input.size() - padding) {
                n += base64Value(input[i + 3]);
                result.push_back(n & 0xFF);
            }
        }
        
        return result;
    }

private:
    static inline uint8_t nibbleToHexChar(uint8_t nibble) {
        return nibble < 10 ? '0' + nibble : 'A' + (nibble - 10);
    }
    
    static inline uint8_t hexCharToNibble(uint8_t c) {
        if(c >= '0' && c <= '9') return c - '0';
        if(c >= 'a' && c <= 'f') return c - 'a' + 10;
        if(c >= 'A' && c <= 'F') return c - 'A' + 10;
        return 0xFF;  // Invalid
    }
    
    static inline uint8_t base64Char(uint8_t value) {
        static const uint8_t chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        return chars[value];
    }
    
    static inline uint8_t base64Value(uint8_t c) {
        if(c >= 'A' && c <= 'Z') return c - 'A';
        if(c >= 'a' && c <= 'z') return c - 'a' + 26;
        if(c >= '0' && c <= '9') return c - '0' + 52;
        if(c == '+') return 62;
        if(c == '/') return 63;
        return 0;
    }
};

#endif // SERIALCONVERTER_H 