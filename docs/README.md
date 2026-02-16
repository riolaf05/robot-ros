# Documentazione Robot ROS2

Benvenuto nella documentazione completa del progetto robot mobile con ROS2 Humble.

## 📚 Indice Documentazione

### 🚀 Guide Rapide

| Documento | Descrizione | Tempo |
|-----------|-------------|-------|
| [QUICKSTART.md](../QUICKSTART.md) | Guida rapida per avvio veloce | 30 min |

### 🔌 Hardware e Collegamenti

| Documento | Descrizione | Livello |
|-----------|-------------|---------|
| [wiring-diagram.html](wiring-diagram.html) | **🌐 Schema Interattivo HTML** (⭐ Raccomandato) | Visuale |
| [WIRING.md](WIRING.md) | Guida testuale completa con tabelle | Base |
| [WIRING_VISUAL.md](WIRING_VISUAL.md) | Schemi visuali ASCII dettagliati pin-to-pin | Dettagliato |

**Cosa trovi in wiring-diagram.html (⭐ NUOVO):**
- 🎨 Schemi SVG interattivi e colorati
- 🖱️ Hover effects su componenti e collegamenti
- 🌈 Legenda colori per identificare rapidamente i cavi
- 📋 Checklist interattiva con salvataggio automatico
- 📑 Tab per visualizzare singole sezioni
- 🖨️ Funzione stampa ottimizzata

**Cosa trovi in WIRING_VISUAL.md:**
- ✅ Diagrammi ASCII art per ogni componente
- ✅ Pinout dettagliato Arduino Nano
- ✅ Schema collegamenti L298N Motor Driver
- ✅ Collegamenti Encoder con interrupt
- ✅ Sistema alimentazione completo
- ✅ Checklist collegamento passo-passo
- ✅ Troubleshooting visivo con LED

### 🚢 Deployment e Installazione

| Documento | Descrizione | Livello |
|-----------|-------------|---------|
| [DEPLOYMENT.md](DEPLOYMENT.md) | Guida completa deploy su Raspberry Pi | Completo |
| [INSTALLATION_NOTES.md](INSTALLATION_NOTES.md) | Note su serial e diffdrive_arduino | Tecnico |

### 📝 Riepilogo e Note

| Documento | Descrizione | Tipo |
|-----------|-------------|------|
| [SUMMARY.md](SUMMARY.md) | Riepilogo modifiche e architettura | Overview |

## 🗺️ Percorso Consigliato

### Per chi inizia:

1. **Leggi:** [QUICKSTART.md](../QUICKSTART.md) per overview rapido
2. **Monta hardware:** [WIRING_VISUAL.md](WIRING_VISUAL.md) per schemi dettagliati
3. **Deploy:** [DEPLOYMENT.md](DEPLOYMENT.md) per installazione completa
4. **Troubleshooting:** Cerca nella sezione troubleshooting di ogni guida

### Per chi ha già assemblato:

1. **Verifica collegamenti:** [WIRING_VISUAL.md](WIRING_VISUAL.md) - Checklist
2. **Installa software:** [DEPLOYMENT.md](DEPLOYMENT.md) - Sezione "Installazione Pacchetti"
3. **Test sistema:** [DEPLOYMENT.md](DEPLOYMENT.md) - Sezione "Test e Verifica"

### Per chi ha problemi:

1. **Errore serial/diffdrive_arduino:** [INSTALLATION_NOTES.md](INSTALLATION_NOTES.md)
2. **Hardware non funziona:** [WIRING_VISUAL.md](WIRING_VISUAL.md) - Troubleshooting Visivo
3. **Software non funziona:** [DEPLOYMENT.md](DEPLOYMENT.md) - Troubleshooting
4. **Architettura generale:** [SUMMARY.md](SUMMARY.md)

## 📌 Link Rapidi per Problemi Comuni

### Problema: Compilazione diffdrive_arduino fallisce
→ [INSTALLATION_NOTES.md](INSTALLATION_NOTES.md) - Workspace unificato

### Problema: Arduino non comunica
→ [DEPLOYMENT.md](DEPLOYMENT.md#problema-arduino-non-comunica)

### Problema: Encoder non funziona
→ [WIRING_VISUAL.md](WIRING_VISUAL.md#encoder-collegamenti-dettagliati)
→ [DEPLOYMENT.md](DEPLOYMENT.md#problema-encoder-non-funziona)

### Problema: Motori non si muovono
→ [WIRING.md](WIRING.md#motori-non-si-muovono)
→ [WIRING_VISUAL.md](WIRING_VISUAL.md#troubleshooting-visivo)

### Problema: LIDAR non funziona
→ [DEPLOYMENT.md](DEPLOYMENT.md#problema-lidar-non-funziona)

### Problema: Controller Manager non si avvia
→ [DEPLOYMENT.md](DEPLOYMENT.md#problema-controller-manager-non-si-avvia)
→ [INSTALLATION_NOTES.md](INSTALLATION_NOTES.md)

## 🔧 Specifiche Hardware

| Componente | Specifiche | Note |
|------------|-----------|------|
| Raspberry Pi 4 | 2GB+ RAM, Ubuntu 22.04 | Consigliato 4GB |
| Arduino Nano | ATmega328P | Programmato con sketch incluso |
| L298N | Driver motori DC | Rimuovi jumper 5V |
| Motori DC | 12V, 1-2A | Con encoder integrato |
| Encoder | 11 CPR × riduttore | Configurabile nello sketch |
| RPLIDAR | A1/A2 | Alimentato via USB |
| Batteria | 12V LiPo 3S | 2200-5000mAh |

## 📖 Versioni Software

| Software | Versione | Repository |
|----------|----------|-----------|
| ROS2 | Humble | [ros.org](https://docs.ros.org/en/humble/) |
| Ubuntu | 22.04 LTS | [ubuntu.com](https://ubuntu.com/) |
| serial | ROS2 | [RoverRobotics-forks/serial-ros2](https://github.com/RoverRobotics-forks/serial-ros2) |
| diffdrive_arduino | Latest | [joshnewans/diffdrive_arduino](https://github.com/joshnewans/diffdrive_arduino) |

## 🤝 Contributi

Per segnalare problemi o contribuire:
- Issues: https://github.com/riolaf05/robot-ros/issues
- Pull Requests: Benvenute!

## 📅 Aggiornamenti

- **2026-02-16:** Aggiunto WIRING_VISUAL.md con schemi dettagliati
- **2026-02-15:** Aggiunto INSTALLATION_NOTES.md per troubleshooting serial
- **2026-02-14:** Aggiornamento completo sistema ros2_control

---

**Ultima revisione:** 2026-02-16
**Versione documentazione:** 1.1
