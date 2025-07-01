# Changelog

## [v2.3] - 2025-07-01

### Added
- Archivo `config.json` para configuración persistente del sistema.
- Modo de calibración interactiva de sensores de humedad vía Bluetooth.
- Detención de saturación en sensores de suelo con reporte SAT-H y SAT-S.
- Preguntas interactivas por Bluetooth para editar:
  - Intervalo de sueño.
  - Número de teléfono para SMS.
  - Sincronización horaria.
- Inclusión del nombre del equipo (`nombre_equipo`) en SMS y encabezado del log CSV.
- Modularización de inicialización: funciones independientes para RTC, SD, Modbus, ADS1115, SIM800L.
- Nuevas funciones: `leerPromedioSuelo()`, `calcularPorcentajeHumedad()`, `calibrarSensoresSuelo()`, entre otras.

### Changed
- `setup()` reorganizado: inicializa periféricos solo si están habilitados en config.
- Ahora todos los parámetros configurables (APN, número SMS, tiempos, flags) están en `config.json`.
- Se mejoró la estructura del log: encabezado dinámico, nombre de archivo configurable.
- Manejo de errores más robusto: SMS para fallos medios, deep sleep para fallos críticos.
- Mejora visual: parpadeo de LEDs de estado para feedback en Bluetooth.
- Eliminado reinicio en fallos críticos repetidos; ahora se entra en deep sleep indefinido.
- Optimización de reintentos al enviar SMS, configurable por usuario.

### Removed
- Variable `falloCriticoPrevio` en RTC_NOINIT_ATTR.
- Dependencia rígida de EEPROM y parámetros hardcodeados.

---

## [v2.1] - Anterior
- Versión base con estructura monolítica.
- Parámetros embebidos en el código.
- Sin calibración interactiva ni configuración dinámica por Bluetooth.
- Log básico en SD sin validación de encabezado ni personalización.

