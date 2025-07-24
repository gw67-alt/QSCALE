// RMS Peak Detector with SHA-256 Hashing
// Scalable multi-channel and multi-hash implementation with hash rate tracking

#include "sha256.h"

Sha256 sha256;

// Configuration - easily adjustable
#define NUM_CHANNELS 4        // Number of analog channels to use
#define NUM_HASH_TARGETS 4    // Number of different hash targets per cycle
#define SAMPLE_RATE_MS 1     // 50Hz sampling
#define HISTORY_SIZE 10       // Samples for RMS calculation

// Channel configuration
int analogPins[NUM_CHANNELS] = {A0, A1, A2, A3};
float channelHistory[NUM_CHANNELS][HISTORY_SIZE];
int historyIndex[NUM_CHANNELS];

// Hash target configuration
struct HashTarget {
  long nonce;
  long increment;
  const char* prefix;
};

HashTarget hashTargets[NUM_HASH_TARGETS] = {
  {0, 1, "GeorgeW"},           // Target 0: increment by +1
  {9999999, -1, "GeorgeW"},    // Target 1: decrement by -1  
  {99999999, -1, "GeorgeW"},   // Target 2: decrement by -1
  {999999999, -1, "GeorgeW"}   // Target 3: decrement by -1
};

unsigned long lastSample = 0;
unsigned long cost = 10000000;
unsigned long STATE_SPACE = 99999999;

// State variables
float maxRmsEver = 0.0;
long QuantumVolume = 0;
unsigned long hashesFound = 0;

// Hash rate tracking variables
unsigned long totalHashesComputed = 0;
unsigned long lastHashRateUpdate = 0;
unsigned long hashesAtLastUpdate = 0;
float currentHashRate = 0.0;
float averageHashRate = 0.0;
unsigned long programStartTime = 0;

unsigned long lastSpeedupPrint = 0;
#define SPEEDUP_INTERVAL 5000
#define HASH_RATE_UPDATE_INTERVAL 1000  // Update hash rate every 1 second
#define DIFFICULTY 1

void setup() {
  Serial.begin(115200);

  // Initialize timing
  programStartTime = millis();
  lastHashRateUpdate = programStartTime;

  // Initialize all channel history buffers and indices
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    historyIndex[ch] = 0;
    for (int i = 0; i < HISTORY_SIZE; i++) {
      channelHistory[ch][i] = 0.0;
    }
  }

  Serial.println("Multi-Channel RMS Peak Detector with SHA-256 Hashing Initialized.");
  Serial.print("Channels: ");
  Serial.print(NUM_CHANNELS);
  Serial.print(" | Hash Targets: ");
  Serial.println(NUM_HASH_TARGETS);
  Serial.println("Waiting for a new maximum RMS value...");
  Serial.println("Format: Nonce | Quantum Volume | Hash Rate | Speedup | Credits");
  Serial.println("==================================================================");
}

bool checkDifficulty(uint8_t* hash) {
  for (int i = 0; i < DIFFICULTY; i++) {
    if (hash[i] != 0x00) {
      return false;
    }
  }
  return true;
}

void updateHashRate() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastHashRateUpdate >= HASH_RATE_UPDATE_INTERVAL) {
    unsigned long timeDelta = currentTime - lastHashRateUpdate;
    unsigned long hashesDelta = totalHashesComputed - hashesAtLastUpdate;
    
    // Calculate current hash rate (hashes per second)
    currentHashRate = (float)hashesDelta / (timeDelta / 1000.0);
    
    // Calculate average hash rate since program start
    unsigned long totalTime = currentTime - programStartTime;
    if (totalTime > 0) {
      averageHashRate = (float)totalHashesComputed / (totalTime / 1000.0);
    }
    
    // Update tracking variables
    lastHashRateUpdate = currentTime;
    hashesAtLastUpdate = totalHashesComputed;
  }
}

void printSpeedupMetrics() {
  updateHashRate(); // Ensure hash rate is current
  
  float speedup = 0.0;
  const char* speedupStatus = "";
  
  if (QuantumVolume > 0 && hashTargets[0].nonce > 0) {
    speedup = (float)hashTargets[0].nonce / (float)QuantumVolume;
    
    if (speedup > 10.0) {
      speedupStatus = " QUANTUM BOOST";
    } else if (speedup > 5.0) {
      speedupStatus = " HIGH SPEED";
    } else if (speedup > 2.0) {
      speedupStatus = " ACCELERATED";
    } else if (speedup > 1.0) {
      speedupStatus = " NORMAL";
    } else if (speedup > 0.5) {
      speedupStatus = " SLOW";
    } else {
      speedupStatus = " VERY SLOW";
    }
  }
  
  Serial.println("=== PERFORMANCE METRICS ===");
  
  // Print hash rate information
  Serial.print("Hash Rate: ");
  Serial.print(currentHashRate, 1);
  Serial.print(" H/s (Current) | ");
  Serial.print(averageHashRate, 1);
  Serial.println(" H/s (Average)");
  
  Serial.print("Total Hashes: ");
  Serial.print(totalHashesComputed);
  Serial.print(" | Runtime: ");
  Serial.print((millis() - programStartTime) / 1000);
  Serial.println(" seconds");
  
  // Print all hash target nonces
  Serial.print("Nonces - ");
  for (int i = 0; i < NUM_HASH_TARGETS; i++) {
    Serial.print("T");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(hashTargets[i].nonce);
    if (i < NUM_HASH_TARGETS - 1) Serial.print(" | ");
  }
  Serial.println();
  
  Serial.print("QV: ");
  Serial.print(QuantumVolume);
  Serial.print(" | Hashes found: ");
  Serial.print(hashesFound);
  if (QuantumVolume > 0) {
    Serial.print(" | Ratio: ");
    Serial.print(speedup, 2);
    Serial.print("x");
  }
  Serial.println();
  
  if (speedup > 0) {
    Serial.print("Status: ");
    Serial.println(speedupStatus);
    
    if (QuantumVolume > 0) {
      float efficiency = ((float)QuantumVolume / (float)hashTargets[0].nonce) * 100.0;
      Serial.print("Efficiency: ");
      Serial.print(efficiency, 1);
      Serial.print("% | Success Rate: ");
      
      if (totalHashesComputed > 0) {
        float successRate = ((float)hashesFound / (float)totalHashesComputed) * 100.0;
        Serial.print(successRate, 4);
        Serial.println("%");
      } else {
        Serial.println("0.0000%");
      }
    }
  }
  
  Serial.print("Credits: ");
  Serial.println(cost);
  Serial.println("===========================");
}

// Sample all channels
void sampleAllChannels() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    int rawValue = analogRead(analogPins[ch]);
    channelHistory[ch][historyIndex[ch]] = (rawValue / 1023.0) * 5.0;
    historyIndex[ch] = (historyIndex[ch] + 1) % HISTORY_SIZE;
  }
}

// Calculate RMS for a specific channel
float getRMSValue(int channel = 0) {
  if (channel >= NUM_CHANNELS) return 0.0;
  
  float sum = 0;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    sum += channelHistory[channel][i] * channelHistory[channel][i];
  }
  return sqrt(sum / HISTORY_SIZE);
}

// Update all nonce counters
void updateNonceCounters() {
  for (int i = 0; i < NUM_HASH_TARGETS; i++) {
    hashTargets[i].nonce += hashTargets[i].increment;
  }
}

// Perform hashing for all targets
bool performAllHashing() {
  bool winningHashFound = false;
  
  for (int i = 0; i < NUM_HASH_TARGETS; i++) {
    char dataToHash[64];
    sha256.init();
    snprintf(dataToHash, sizeof(dataToHash), "%s%ld", hashTargets[i].prefix, hashTargets[i].nonce);
    sha256.print(dataToHash);
    uint8_t* hashResult = sha256.result();
    totalHashesComputed += 1;

    // Increment total hash counter
    
    if (checkDifficulty(hashResult)) {
      if (!winningHashFound) {
        Serial.println(" === WINNING HASH FOUND === ");
        Serial.print("    Target ");
        Serial.print(i);
        Serial.print(" | Nonce: ");
        Serial.print(hashTargets[i].nonce);
        Serial.print(" | Data: ");
        Serial.println(dataToHash);
        Serial.print("    Hash: ");
        printHash(hashResult);
        
        hashesFound++;
        
        // Update hash rate for this moment
        updateHashRate();
        
        float instantSpeedup = (QuantumVolume > 0) ? (float)hashTargets[0].nonce / (float)QuantumVolume : 0;
        Serial.print("    Quantum Volume: ");
        Serial.print(QuantumVolume);
        Serial.print(" | Hashes found: ");
        Serial.print(hashesFound);
        Serial.print(" | Hash Rate: ");
        Serial.print(currentHashRate, 1);
        Serial.print(" H/s");
        Serial.print(" | Speedup: ");
        Serial.print(instantSpeedup, 2);
        Serial.println("x");
        
        cost += 150;
        Serial.println("    +150 credits earned!");
        winningHashFound = true;
      }
    }
  }
  
  return winningHashFound;
}

void loop() {
  for (int i = 0; i < STATE_SPACE; i++) {
    unsigned long currentTime = millis();
    
    // Update hash rate periodically
    updateHashRate();
    
    // Print speedup metrics periodically
    if (currentTime - lastSpeedupPrint >= SPEEDUP_INTERVAL) {
      printSpeedupMetrics();
      lastSpeedupPrint = currentTime;
    }
    
    float currentRms = getRMSValue(); // Default to channel 0

    if (currentTime - lastSample >= SAMPLE_RATE_MS) {
      lastSample = currentTime;
      
      // Update all nonce counters
      updateNonceCounters();
      
      // Sample all configured channels
      sampleAllChannels();
      QuantumVolume++;

      // Check for new maximum RMS and perform hash if found
      if (currentRms > maxRmsEver && cost > 0) {
        
        // Perform hashing for all targets
        bool winFound = performAllHashing();
        
        if (!winFound) {
          cost -= 1;
          
          // Show compact progress with speedup and hash rate
          if (hashTargets[0].nonce % 100 == 0) {
            float currentSpeedup = (QuantumVolume > 0) ? (float)hashTargets[0].nonce / (float)QuantumVolume : 0;
            Serial.print("N:");
            Serial.print(hashTargets[0].nonce*NUM_HASH_TARGETS);
            Serial.print(" QV:");
            Serial.print(QuantumVolume);
            Serial.print(" H/s:");
            Serial.print(currentHashRate, 0);
            Serial.print(" SP:");
            Serial.print(currentSpeedup, 1);
            Serial.print("x C:");
            Serial.println(cost);
          }
        }
        
        // Update the max RMS
        maxRmsEver = currentRms;
        if (currentRms == maxRmsEver) {
          maxRmsEver /= 10;
        }
      }
    }
  }
}

void printHash(uint8_t* hash) {
  for (int i = 0; i < 32; i++) {
    if (hash[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(hash[i], HEX);
  }
  Serial.println();
}

// Helper function to get current hash rate
float getCurrentHashRate() {
  updateHashRate();
  return currentHashRate;
}

// Helper function to get average hash rate
float getAverageHashRate() {
  updateHashRate();
  return averageHashRate;
}

// Helper function to get total hashes computed
unsigned long getTotalHashes() {
  return totalHashesComputed;
}

// Helper function to add a new channel (call in setup() if needed)
void addChannel(int pin) {
  if (NUM_CHANNELS < 8) { // Arduino Uno has limited analog pins
    analogPins[NUM_CHANNELS] = pin;
    // Note: You'd need to increase NUM_CHANNELS and recompile
    Serial.print("Channel added: A");
    Serial.println(pin);
  }
}

// Helper function to add a new hash target (call in setup() if needed)
void addHashTarget(long startNonce, long increment, const char* prefix) {
  // Note: You'd need to increase NUM_HASH_TARGETS and recompile
  Serial.print("Hash target would be added: ");
  Serial.print(prefix);
  Serial.print(" starting at ");
  Serial.print(startNonce);
  Serial.print(" increment ");
  Serial.println(increment);
}
