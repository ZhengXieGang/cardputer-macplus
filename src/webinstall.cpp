#include "debug_log.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <M5Cardputer.h>
#include <esp_spi_flash.h>
#include <new>
#include <errno.h>
#include <sys/stat.h>

#include "hardware_config.h"
#include "input.h"
#include "settings.h"
#include "webinstall.h"
#include "sdcard.h"
#include "storage_layout.h"

extern "C" {
#include "tme/disp.h"
#include "tme/hd.h"
#include "tme/snd.h"
}

static constexpr uint32_t WEB_IMAGE_MIN_BYTES = 0xC600;
static constexpr uint32_t WEB_PINNED_MAGIC = 0x4D414357; // "MACW"
static constexpr uint32_t INSTALL_400K_BYTES =
    MACPLUS_INSTALL_400K_BYTES;
static constexpr uint32_t INSTALL_800K_BYTES =
    MACPLUS_INSTALL_800K_BYTES;
static constexpr const char *INSTALL_PATH = "/sd/macplus-install.img";
static constexpr const char *INSTALL_TEMP_PATH = "/sd/macplus-install.upload";
static constexpr const char *INSTALL_BACKUP_PATH = "/sd/macplus-install.backup";

static WebServer *webServer = nullptr;
static bool uploadError = false;
static bool uploadFinished = false;
enum class UploadTarget : int {
    Invalid = -1,
    HardDisk = 0,
    InstallDisk = 1,
};
enum class UploadProgressStage : uint8_t {
    Invalid,
    Erase,
    Write,
};
static UploadTarget uploadTarget = UploadTarget::Invalid;
static uint32_t uploadOffset = 0;
static uint32_t uploadContentCrc = 0;
static uint32_t uploadExpectedBytes = 0;
static uint32_t uploadRequestBytes = 0;
static uint32_t uploadInputOffset = 0;
static bool uploadIsDc42 = false;
static uint8_t dc42Header[84];
static FILE *sdFile = nullptr;
static bool accessPointReady = false;
static bool accessPointAttempted = false;
static uint32_t lastDisplayUpdateMs = 0;

static const char INSTALL_PAGE_LEGACY[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>MacPlus Transfer</title><style>*{box-sizing:border-box}html,body{margin:0;min-height:100%;color:#111;background:#aaa;font:14px Geneva,Chicago,"Helvetica Neue",Arial,sans-serif;letter-spacing:0}button,input{font:inherit}.menu{height:30px;display:flex;align-items:center;gap:22px;padding:0 12px;background:#fff;border-bottom:2px solid #000}.menu b{font-size:16px}.menu .lang{margin-left:auto;font-size:12px}.desktop{min-height:calc(100vh - 30px);padding:5vh 16px}.window{width:min(640px,100%);margin:auto;background:#fff;border:2px solid #000;box-shadow:7px 7px 0 #555;animation:open .18s ease-out}.titlebar{height:31px;display:grid;grid-template-columns:28px 1fr 28px;align-items:center;border-bottom:2px solid #000;text-align:center}.titlebar h1{margin:0;font-size:14px}.close{width:15px;height:15px;margin:auto;border:2px solid #000}.body{padding:20px 22px}.intro{margin:0 0 16px;line-height:1.45}.online{display:flex;align-items:center;gap:8px;margin-bottom:8px;font-weight:bold}.lamp{width:10px;height:10px;background:#111;border:1px solid #000}.disk{padding:17px 0;border-top:1px solid #000}.disk h2{margin:0 0 5px;font-size:17px}.disk p{margin:0 0 13px;color:#333}.actions{display:grid;grid-template-columns:auto minmax(100px,1fr) auto;gap:10px;align-items:center}.pick,.send{min-height:34px;padding:7px 13px;background:#fff;color:#000;border:2px solid #000;border-radius:0;box-shadow:2px 2px 0 #000;cursor:pointer}.pick input{position:absolute;width:1px;height:1px;opacity:0}.pick:active,.send:active{transform:translate(2px,2px);box-shadow:none}.send:disabled{color:#777;border-color:#777;box-shadow:2px 2px 0 #777;cursor:not-allowed}.filename{min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.monitor{padding-top:17px;border-top:2px solid #000}.monitor-head{display:flex;justify-content:space-between;margin-bottom:8px}.bar{height:20px;padding:3px;border:2px solid #000;background:#fff}.bar span{display:block;width:0;height:100%;background:#111;transition:width .12s linear}.message{min-height:20px;margin:8px 0 0}.note{margin:15px 0 0;font-size:12px;color:#333}@media(max-width:520px){.desktop{padding:18px 10px}.body{padding:16px}.actions{grid-template-columns:1fr auto}.filename{grid-column:1/-1;grid-row:2}.pick,.send{width:100%}}@media(prefers-reduced-motion:reduce){.window{animation:none}.bar span{transition:none}}@keyframes open{from{transform:scale(.98);opacity:.45}to{transform:scale(1);opacity:1}}</style></head><body><header class="menu"><b>MacPlus</b><span data-t="menu">File Transfer</span><span class="lang" id="lang">English</span></header><main class="desktop"><section class="window" aria-labelledby="title"><header class="titlebar"><span class="close" aria-hidden="true"></span><h1 id="title" data-t="title">MacPlus Transfer</h1><span></span></header><div class="body"><p class="intro" data-t="intro">Upload software disks directly to this Cardputer.</p><div class="online"><span class="lamp"></span><span data-t="online">Connected to MacPlus-Install</span></div><form class="disk" action="/upload/hd" data-min="50688" data-max="0" data-step="512"><h2 data-t="hd">System Hard Disk</h2><p data-t="hdInfo">Reading available Flash capacity...</p><div class="actions"><label class="pick"><input type="file" name="hdimg" accept=".img"><span data-t="choose">Choose Image</span></label><span class="filename" data-t="none">No file selected</span><button class="send" type="submit" data-t="upload">Upload</button></div></form><form class="disk" action="/upload/install" data-sizes="409600,819200,409684,419284,819284,838484"><h2 data-t="install">Software Disk</h2><p data-t="installInfo">400K or 800K HFS/MFS image, raw or Disk Copy 4.2</p><div class="actions"><label class="pick"><input type="file" name="installimg" accept=".img,.dsk,.image,.dc42"><span data-t="choose">Choose Image</span></label><span class="filename" data-t="none">No file selected</span><button class="send" type="submit" data-t="upload">Upload</button></div></form><section class="monitor" aria-live="polite"><div class="monitor-head"><b data-t="status">Transfer Status</b><output id="percent">0%</output></div><div class="bar" id="bar" role="progressbar" aria-valuemin="0" aria-valuemax="100" aria-valuenow="0"><span id="fill"></span></div><p class="message" id="message"></p></section><p class="note" data-t="note">The device verifies the image and restarts automatically after a successful upload.</p></div></section></main><script>const T={en:{menu:"File Transfer",title:"MacPlus Transfer",intro:"Upload disk images directly to this Cardputer.",online:"Connected to MacPlus-Install",hd:"System Hard Disk",hdInfo:"Macintosh partition image, 512-byte aligned, up to {max} bytes on this device",install:"Software Disk",installInfo:"400K or 800K HFS/MFS image, raw or Disk Copy 4.2",choose:"Choose Image",none:"No file selected",upload:"Upload",status:"Transfer Status",note:"After a software-disk upload, open its Finder icon and copy the app to the system disk.",ready:"Ready.",badHd:"Invalid hard disk image size. Use 512-byte alignment from {min} to {max} bytes.",badInstall:"Software disk must be 400K (409,600) or 800K (819,200), raw or Disk Copy 4.2.",prepare:"Preparing storage...",sending:"Uploading {name}: {percent}%",verify:"Upload sent. Verifying...",done:"Complete. Restarting MacPlus...",fail:"Upload failed. Check the image and try again."},zh:{menu:"文件传输",title:"MacPlus 传输工具",intro:"把磁盘镜像直接上传到这台 Cardputer。",online:"已连接 MacPlus-Install",hd:"系统硬盘",hdInfo:"Macintosh 分区硬盘镜像，512 字节对齐；本机最大 {max} 字节",install:"软件安装盘",installInfo:"支持 400K 或 800K HFS/MFS 原始镜像或 Disk Copy 4.2 文件",choose:"选择镜像",none:"未选择文件",upload:"上传",status:"传输状态",note:"上传软件盘后，在 Finder 打开新磁盘图标，把应用复制到系统硬盘。",ready:"准备就绪。",badHd:"系统硬盘镜像大小无效，须为 512 字节对齐，范围 {min} 至 {max} 字节。",badInstall:"软件盘必须是 400K（409,600 字节）或 800K（819,200 字节），支持原始镜像或 Disk Copy 4.2。",prepare:"正在准备存储空间...",sending:"正在上传 {name}：{percent}%",verify:"上传完成，正在校验...",done:"写入完成，正在重启 MacPlus...",fail:"上传失败，请检查镜像后重试。"}};const lang=(navigator.language||"").toLowerCase().startsWith("zh")?"zh":"en",t=T[lang],q=s=>document.querySelector(s),all=s=>[...document.querySelectorAll(s)];document.documentElement.lang=lang==="zh"?"zh-CN":"en";q("#lang").textContent=lang==="zh"?"简体中文":"English";all("[data-t]").forEach(e=>e.textContent=t[e.dataset.t]);const bar=q("#bar"),fill=q("#fill"),pct=q("#percent"),message=q("#message"),forms=all("form.disk");function setProgress(value,text){value=Math.max(0,Math.min(100,value));fill.style.width=value+"%";pct.textContent=value+"%";bar.setAttribute("aria-valuenow",value);message.textContent=text}function enableForms(on){forms.forEach(f=>f.querySelector(".send").disabled=!on||!f.dataset.valid)}setProgress(0,t.ready);forms.forEach(form=>{const input=form.querySelector("input"),name=form.querySelector(".filename"),button=form.querySelector(".send"),sizes=(form.dataset.sizes||"").split(",").filter(Boolean).map(Number),min=Number(form.dataset.min||0),step=Number(form.dataset.step||1);let max=Number(form.dataset.max||0);const valid=file=>!!file&&(sizes.length?sizes.includes(file.size):max>0&&file.size>=min&&file.size<=max&&file.size%step===0),bad=()=>sizes.length?t.badInstall:t.badHd.replace("{min}",min.toLocaleString()).replace("{max}",max.toLocaleString());form.dataset.valid="";button.disabled=true;if(!sizes.length)fetch("/info").then(r=>r.json()).then(info=>{max=Number(info.hdMax||0);form.dataset.max=max;form.querySelector("p").textContent=t.hdInfo.replace("{max}",max.toLocaleString());input.dispatchEvent(new Event("change"))});input.addEventListener("change",()=>{const file=input.files[0],ok=valid(file);name.textContent=file?file.name:t.none;form.dataset.valid=ok?"1":"";button.disabled=!ok;setProgress(0,file&&!ok?bad():t.ready)});form.addEventListener("submit",event=>{event.preventDefault();const file=input.files[0];if(!valid(file))return;enableForms(false);setProgress(0,t.prepare);const xhr=new XMLHttpRequest();xhr.open("POST",form.action);xhr.setRequestHeader("Content-Type","application/octet-stream");xhr.upload.onprogress=e=>{if(!e.lengthComputable)return;const p=Math.round(e.loaded*100/e.total);setProgress(p,t.sending.replace("{name}",file.name).replace("{percent}",p))};xhr.upload.onload=()=>setProgress(100,t.verify);xhr.onload=()=>{if(xhr.status>=200&&xhr.status<300)setProgress(100,t.done);else{setProgress(0,t.fail);enableForms(true)}};xhr.onerror=()=>{setProgress(0,t.fail);enableForms(true)};xhr.send(file)})});</script></body></html>)HTML";
static uint32_t webCrc32(const uint8_t *data, size_t len, uint32_t crc) {
    crc = ~crc;
    while (len--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static uint32_t installVolumeBytesForUpload(uint32_t bytes) {
    if (bytes == INSTALL_400K_BYTES || bytes == INSTALL_800K_BYTES) {
        return bytes;
    }
    if (bytes == INSTALL_400K_BYTES + 84U ||
        bytes == INSTALL_400K_BYTES + 84U + 800U * 12U) {
        return INSTALL_400K_BYTES;
    }
    if (bytes == INSTALL_800K_BYTES + 84U ||
        bytes == INSTALL_800K_BYTES + 84U + 1600U * 12U) {
        return INSTALL_800K_BYTES;
    }
    return 0;
}

static bool uploadSizeIsValid(UploadTarget target, uint32_t bytes) {
    if (target == UploadTarget::HardDisk) {
        const uint32_t maxBytes = hdRawFlashStorageIsSafe(512U)
            ? hdRawFlashStorageMaxImageBytes() & ~511U : 0;
        return bytes >= WEB_IMAGE_MIN_BYTES &&
               bytes <= maxBytes && (bytes % 512U) == 0;
    }
    return installVolumeBytesForUpload(bytes) != 0;
}

static uint32_t readBe32(const uint8_t *data) {
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) | data[3];
}

static bool dc42HeaderIsValid(uint32_t volumeBytes, uint32_t requestBytes) {
    if (dc42Header[0] > 63 || readBe32(dc42Header + 64) != volumeBytes) {
        return false;
    }
    const uint32_t tagBytes = readBe32(dc42Header + 68);
    return requestBytes == 84U + volumeBytes + tagBytes &&
           (tagBytes == 0 || tagBytes == (volumeBytes / 512U) * 12U);
}

static const char *uploadTargetName(UploadTarget target) {
    return target == UploadTarget::HardDisk ? "HD" : "INSTALL";
}

// The transfer mode is usable without a phone: the four punctuation keys are
// a small D-pad and Enter/GO activates the selected row.  Keep this UI in the
// firmware so the web page only has to handle the two image uploads.
enum class WifiMenuItem : uint8_t {
    Sensitivity,
    Volume,
    ClearHd,
    RemoveFloppy,
};

static constexpr uint8_t WIFI_KEY_ENTER = 0x24;
static constexpr uint8_t WIFI_KEY_UP = 0x29;       // ;
static constexpr uint8_t WIFI_KEY_LEFT = 0x2B;     // ,
static constexpr uint8_t WIFI_KEY_RIGHT = 0x2C;    // /
static constexpr uint8_t WIFI_KEY_DOWN = 0x2F;     // .
static WifiMenuItem wifiMenuItem = WifiMenuItem::Sensitivity;
static char wifiMenuStatus[40] = "[RUN] STARTING AP";

static uint8_t wifiVolumePercent(void) {
    return static_cast<uint8_t>((static_cast<unsigned>(sndGetVolume()) * 100U +
                                 127U) / 255U);
}

static bool saveWifiSettings(uint16_t pointerSpeedPercent, uint8_t percent) {
    const uint8_t volume = static_cast<uint8_t>(
        (static_cast<unsigned>(percent) * 255U + 50U) / 100U);
    return saveMacSettings(pointerSpeedPercent, volume);
}

static void renderWifiMenu(void) {
    char speed[40];
    char volume[32];
    char clearHd[40];
    char removeFloppy[40];
    snprintf(speed, sizeof(speed), "%c POINTER SPEED %u%%",
             wifiMenuItem == WifiMenuItem::Sensitivity ? '>' : ' ',
             static_cast<unsigned>(imuPointerSpeedPercent));
    snprintf(volume, sizeof(volume), "%c VOLUME (SPEAKER) %u%%",
             wifiMenuItem == WifiMenuItem::Volume ? '>' : ' ',
             static_cast<unsigned>(wifiVolumePercent()));
    snprintf(clearHd, sizeof(clearHd), "%c CLEAR FLASH DISK CACHE",
             wifiMenuItem == WifiMenuItem::ClearHd ? '>' : ' ');
    snprintf(removeFloppy, sizeof(removeFloppy), "%c DELETE SD SOFTWARE DISK",
             wifiMenuItem == WifiMenuItem::RemoveFloppy ? '>' : ' ');
    const char *lines[] = {
        "WIFI TRANSFER MODE",
        accessPointReady ? "[OK] AP MacPlus-Install" :
            (accessPointAttempted ? "[FAIL] AP START ERROR" :
                                     "[RUN] STARTING AP"),
        "IP 192.168.4.1",
        speed,
        volume,
        clearHd,
        removeFloppy,
        wifiMenuStatus,
    };
    dispShowMessage(lines, 8);
}

static void showTransferStatus(const char *status) {
    snprintf(wifiMenuStatus, sizeof(wifiMenuStatus), "%s",
             status != nullptr ? status : "[OK] READY FOR UPLOAD");
    renderWifiMenu();
}

static void adjustWifiMenu(int direction) {
    if (wifiMenuItem == WifiMenuItem::Sensitivity) {
        int value = static_cast<int>(imuPointerSpeedPercent) +
                    direction * IMU_POINTER_SPEED_STEP_PERCENT;
        if (value < IMU_POINTER_SPEED_MIN_PERCENT) {
            value = IMU_POINTER_SPEED_MIN_PERCENT;
        }
        if (value > IMU_POINTER_SPEED_MAX_PERCENT) {
            value = IMU_POINTER_SPEED_MAX_PERCENT;
        }
        if (saveWifiSettings(static_cast<uint16_t>(value),
                             wifiVolumePercent())) {
            snprintf(wifiMenuStatus, sizeof(wifiMenuStatus), "[OK] SPEED %d%%",
                     value);
        } else {
            snprintf(wifiMenuStatus, sizeof(wifiMenuStatus),
                     "[FAIL] SETTINGS NVS");
        }
    } else if (wifiMenuItem == WifiMenuItem::Volume) {
        int value = static_cast<int>(wifiVolumePercent()) + direction * 5;
        if (value < 0) value = 0;
        if (value > 100) value = 100;
        if (saveWifiSettings(imuPointerSpeedPercent,
                             static_cast<uint8_t>(value))) {
            snprintf(wifiMenuStatus, sizeof(wifiMenuStatus), "[OK] VOLUME %d%%",
                     value);
        } else {
            snprintf(wifiMenuStatus, sizeof(wifiMenuStatus),
                     "[FAIL] SETTINGS NVS");
        }
    }
    renderWifiMenu();
}

static void activateWifiMenu(void) {
    if (wifiMenuItem == WifiMenuItem::ClearHd) {
        snprintf(wifiMenuStatus, sizeof(wifiMenuStatus),
                 hdInvalidateRawCache() ? "[OK] FLASH DISK CACHE CLEARED" :
                                          "[FAIL] FLASH CACHE UNAVAILABLE");
    } else if (wifiMenuItem == WifiMenuItem::RemoveFloppy) {
        const bool mounted = sdcardMounted() || sdcardInit();
        const bool removed = mounted && hdRemoveInstallImage();
        snprintf(wifiMenuStatus, sizeof(wifiMenuStatus),
                 removed ? "[OK] SD SOFTWARE DISK DELETED" :
                          "[FAIL] SD SOFTWARE DISK ERROR");
    } else {
        snprintf(wifiMenuStatus, sizeof(wifiMenuStatus),
                 "[OK] LEFT/RIGHT ADJUST");
    }
    renderWifiMenu();
}

static void handleWifiMenuKey(uint8_t key) {
    switch (key) {
    case WIFI_KEY_UP:
        wifiMenuItem = static_cast<WifiMenuItem>(
            (static_cast<uint8_t>(wifiMenuItem) + 3U) % 4U);
        renderWifiMenu();
        break;
    case WIFI_KEY_DOWN:
        wifiMenuItem = static_cast<WifiMenuItem>(
            (static_cast<uint8_t>(wifiMenuItem) + 1U) % 4U);
        renderWifiMenu();
        break;
    case WIFI_KEY_LEFT:
        adjustWifiMenu(-1);
        break;
    case WIFI_KEY_RIGHT:
        adjustWifiMenu(1);
        break;
    case WIFI_KEY_ENTER:
        activateWifiMenu();
        break;
    default:
        break;
    }
}

static void showUploadProgress(UploadProgressStage stage,
                               uint32_t current, uint32_t total) {
    const char *stageName = stage == UploadProgressStage::Erase
        ? "ERASE" : "WRITE";
    char status[36];
    const uint32_t percent = total == 0
        ? 0 : min<uint32_t>(100U, static_cast<uint32_t>(
              static_cast<uint64_t>(current) * 100U / total));
    snprintf(status, sizeof(status), "[%s] %s %s",
             stageName, uploadTargetName(uploadTarget),
             uploadTarget == UploadTarget::InstallDisk ? "SD" : "FLASH");
    dispShowProgress("WIFI TRANSFER MODE", status,
                     "AP MacPlus-Install / 192.168.4.1", current, total);

}

static bool hardDiskFingerprint(uint32_t imageBytes, uint32_t *fingerprint) {
    if (fingerprint == nullptr || !uploadSizeIsValid(UploadTarget::HardDisk,
                                                      imageBytes)) {
        return false;
    }
    uint8_t sector[512];
    uint32_t crc = 0;
    if (!hdRawFlashStorageRead(0, sector, sizeof(sector))) {
        return false;
    }
    crc = webCrc32(sector, sizeof(sector), crc);
    if (!hdRawFlashStorageRead(imageBytes - sizeof(sector), sector,
                               sizeof(sector))) {
        return false;
    }
    crc = webCrc32(sector, sizeof(sector), crc);
    *fingerprint = crc ^ imageBytes;
    return true;
}

static bool writeHardDiskMetadata(uint32_t magic, uint32_t imageBytes,
                                  uint32_t fingerprint, uint32_t contentCrc) {
    uint32_t metadata[4] = {magic, imageBytes, fingerprint, contentCrc};
    return spi_flash_write(hdRawFlashStorageMetadataAddress(), metadata,
                           sizeof(metadata)) == ESP_OK;
}

static bool closeSdFile();

static bool writeUploadedData(const uint8_t *data, size_t bytes) {
    if (uploadTarget == UploadTarget::HardDisk) {
        const esp_err_t error = spi_flash_write(
            hdRawFlashStorageAddress() + uploadOffset, data, bytes);
        if (error != ESP_OK) {
            MACPLUS_LOG("WEB: Flash write failed at %lu (%d)\n",
                   static_cast<unsigned long>(uploadOffset),
                   static_cast<int>(error));
            return false;
        }
        uploadContentCrc = webCrc32(data, bytes, uploadContentCrc);
    } else if (sdFile == nullptr || fwrite(data, 1, bytes, sdFile) != bytes) {
        MACPLUS_LOG("WEB: install disk SD write failed\n");
        return false;
    }
    uploadOffset += static_cast<uint32_t>(bytes);
    return true;
}

static bool uploadedImageSignatureIsValid(UploadTarget target) {
    if (target == UploadTarget::HardDisk) {
        return hdRawFlashImageIsValid(uploadExpectedBytes);
    }
    uint8_t signature[2] = {};
    FILE *file = fopen(INSTALL_TEMP_PATH, "rb");
    const bool valid = file != nullptr &&
        fseek(file, 1024L, SEEK_SET) == 0 &&
        fread(signature, 1, sizeof(signature), file) == sizeof(signature) &&
        ((signature[0] == 0x42 && signature[1] == 0x44) ||
         (signature[0] == 0xD2 && signature[1] == 0xD7));
    if (file != nullptr) fclose(file);
    return valid;
}

static bool closeSdFile() {
    bool ok = true;
    if (sdFile != nullptr) {
        ok = fflush(sdFile) == 0;
        ok = fclose(sdFile) == 0 && ok;
        sdFile = nullptr;
    }
    return ok;
}

static bool commitInstallDisk() {
    struct stat originalInfo = {};
    struct stat backupInfo = {};
    if (stat(INSTALL_PATH, &originalInfo) != 0 &&
        stat(INSTALL_BACKUP_PATH, &backupInfo) == 0) {
        if (rename(INSTALL_BACKUP_PATH, INSTALL_PATH) != 0) return false;
    }
    remove(INSTALL_BACKUP_PATH);
    errno = 0;
    const bool hadOriginal = rename(INSTALL_PATH, INSTALL_BACKUP_PATH) == 0;
    if (!hadOriginal && errno != ENOENT) return false;
    if (rename(INSTALL_TEMP_PATH, INSTALL_PATH) != 0) {
        if (hadOriginal) rename(INSTALL_BACKUP_PATH, INSTALL_PATH);
        return false;
    }
    if (hadOriginal) remove(INSTALL_BACKUP_PATH);
    return true;
}

static void handleRoot() {
    webServer->send_P(200, "text/html; charset=utf-8", INSTALL_PAGE_LEGACY);
}

static void handleInfo() {
    const uint32_t maxBytes = hdRawFlashStorageIsSafe(512U)
        ? hdRawFlashStorageMaxImageBytes() & ~511U : 0;
    char response[48];
    snprintf(response, sizeof(response), "{\"hdMax\":%lu}",
             static_cast<unsigned long>(maxBytes));
    webServer->send(200, "application/json", response);
}

static void handleUploadFinish(UploadTarget target) {
    const bool ok = uploadFinished && !uploadError &&
                    uploadTarget == target && uploadExpectedBytes != 0 &&
                    uploadOffset == uploadExpectedBytes;
    if (ok) {
        showTransferStatus("[OK] UPLOAD COMPLETE");
        webServer->send(200, "text/plain; charset=utf-8",
                        "OK, rebooting into MacPlus...\n");
        delay(1500);
        exitWebInstallMode();
    } else {
        showTransferStatus("[FAIL] UPLOAD ERROR");
        webServer->send(400, "text/plain; charset=utf-8",
                        "FAIL: image size or write error\n");
    }
}

static void handleRawUpload(UploadTarget target) {
    HTTPRaw &upload = webServer->raw();
    if (upload.status == RAW_START) {
        closeSdFile();
        uploadError = false;
        uploadFinished = false;
        uploadTarget = target;
        uploadOffset = 0;
        uploadContentCrc = 0;
        uploadExpectedBytes = 0;
        uploadRequestBytes = 0;
        uploadInputOffset = 0;
        uploadIsDc42 = false;

        const long requestLength = webServer->header("Content-Length").toInt();
        const uint32_t requestBytes = requestLength > 0 &&
                                              requestLength <= UINT32_MAX
                                          ? static_cast<uint32_t>(requestLength)
                                          : 0;
        MACPLUS_LOG("WEB: upload start (target %d, %lu bytes)\n",
               static_cast<int>(uploadTarget),
               static_cast<unsigned long>(requestBytes));
        if (!uploadSizeIsValid(target, requestBytes)) {
            MACPLUS_LOG("WEB: rejected size %lu bytes\n",
                   static_cast<unsigned long>(requestBytes));
            uploadError = true;
            uploadFinished = true;
            showTransferStatus("[FAIL] IMAGE SIZE ERROR");
            return;
        }
        uploadRequestBytes = requestBytes;
        uploadExpectedBytes = target == UploadTarget::HardDisk
                                  ? requestBytes
                                  : installVolumeBytesForUpload(requestBytes);
        uploadIsDc42 = target == UploadTarget::InstallDisk &&
                       requestBytes != uploadExpectedBytes;
        if (uploadTarget == UploadTarget::InstallDisk) {
            if (!sdcardMounted()) sdcardInit();
            if (sdcardMounted()) {
                remove(INSTALL_TEMP_PATH);
                sdFile = fopen(INSTALL_TEMP_PATH, "wb");
            }
            if (sdFile == nullptr) {
                MACPLUS_LOG("WEB: cannot create %s\n", INSTALL_TEMP_PATH);
                uploadError = true;
                uploadFinished = true;
                showTransferStatus("[FAIL] SD CARD REQUIRED");
                return;
            }
        } else if (!hdRawFlashStorageIsSafe(uploadExpectedBytes)) {
            MACPLUS_LOG("WEB: '%s' partition is missing or too small\n",
                   MACPLUS_DATA_PARTITION_LABEL);
            uploadError = true;
            uploadFinished = true;
            showTransferStatus("[FAIL] DATA PARTITION REQUIRED");
            return;
        }

        if (target == UploadTarget::HardDisk) {
            const uint32_t imageEraseBytes =
                (uploadExpectedBytes + 4095U) & ~4095U;
            const uint32_t totalEraseBytes =
                MACPLUS_HD_METADATA_BYTES + imageEraseBytes;
            showUploadProgress(UploadProgressStage::Erase, 0, totalEraseBytes);
            if (spi_flash_erase_range(hdRawFlashStorageMetadataAddress(),
                                      MACPLUS_HD_METADATA_BYTES) != ESP_OK) {
                uploadError = true;
            }
            for (uint32_t erased = 0; !uploadError && erased < imageEraseBytes;
                 erased += 0x10000U) {
                const uint32_t left = imageEraseBytes - erased;
                const uint32_t chunk = left < 0x10000U ? left : 0x10000U;
                const esp_err_t err = spi_flash_erase_range(
                    hdRawFlashStorageAddress() + erased, chunk);
                if (err != ESP_OK) {
                    MACPLUS_LOG("WEB: Flash erase failed (%d)\n",
                           static_cast<int>(err));
                    uploadError = true;
                }
                if (!uploadError) {
                    showUploadProgress(UploadProgressStage::Erase,
                        MACPLUS_HD_METADATA_BYTES + erased + chunk,
                        totalEraseBytes);
                }
                delay(1);
            }
        }
        if (uploadError) {
            showTransferStatus("[FAIL] ERASE ERROR");
        } else {
            showUploadProgress(UploadProgressStage::Write, 0,
                               uploadExpectedBytes);
        }
        lastDisplayUpdateMs = millis();
    } else if (upload.status == RAW_WRITE) {
        const uint32_t maxBytes = uploadExpectedBytes;
        if (!uploadError && uploadTarget == target &&
            upload.currentSize <= uploadRequestBytes - uploadInputOffset) {
            const uint8_t *source = upload.buf;
            size_t remaining = upload.currentSize;
            uint32_t inputPosition = uploadInputOffset;
            if (uploadIsDc42 && inputPosition < sizeof(dc42Header)) {
                const size_t headerBytes = min<size_t>(
                    remaining, sizeof(dc42Header) - inputPosition);
                memcpy(dc42Header + inputPosition, source, headerBytes);
                source += headerBytes;
                remaining -= headerBytes;
                inputPosition += static_cast<uint32_t>(headerBytes);
                if (inputPosition == sizeof(dc42Header) &&
                    !dc42HeaderIsValid(maxBytes, uploadRequestBytes)) {
                    MACPLUS_LOG("WEB: invalid Disk Copy 4.2 header\n");
                    uploadError = true;
                }
            }
            if (!uploadError && uploadIsDc42 && remaining != 0 &&
                inputPosition < sizeof(dc42Header) + maxBytes) {
                const size_t dataBytes = min<size_t>(
                    remaining, sizeof(dc42Header) + maxBytes - inputPosition);
                if (inputPosition - sizeof(dc42Header) != uploadOffset ||
                    !writeUploadedData(source, dataBytes)) {
                    uploadError = true;
                }
            } else if (!uploadError && !uploadIsDc42 &&
                       (remaining > maxBytes - uploadOffset ||
                        !writeUploadedData(source, remaining))) {
                uploadError = true;
            }
            uploadInputOffset += upload.currentSize;
            const uint32_t now = millis();
            if (!uploadError && now - lastDisplayUpdateMs >= 250U) {
                lastDisplayUpdateMs = now;
                showUploadProgress(UploadProgressStage::Write,
                                   uploadOffset, maxBytes);
            }
        } else {
            uploadError = true;
        }
    } else if (upload.status == RAW_END) {
        const uint32_t maxBytes = uploadExpectedBytes;
        if (!closeSdFile()) uploadError = true;
        const bool complete = !uploadError && uploadTarget == target &&
                              uploadInputOffset == uploadRequestBytes &&
                              uploadOffset == maxBytes;
        if (complete && !uploadedImageSignatureIsValid(target)) {
            MACPLUS_LOG("WEB: invalid %s image signature\n",
                   uploadTargetName(target));
            uploadError = true;
        }
        if (complete && target == UploadTarget::InstallDisk &&
            !uploadError && !commitInstallDisk()) {
            MACPLUS_LOG("WEB: cannot commit install disk staging file\n");
            uploadError = true;
        }
        if (!uploadError && complete) {
            uint32_t footerCrc = 0;
            if (target == UploadTarget::HardDisk) {
                if (!hardDiskFingerprint(maxBytes, &footerCrc) ||
                    !writeHardDiskMetadata(WEB_PINNED_MAGIC, maxBytes,
                                           footerCrc, uploadContentCrc)) {
                    uploadError = true;
                }
            }
        } else {
            uploadError = true;
        }
        closeSdFile();
        if (uploadError && target == UploadTarget::InstallDisk) {
            remove(INSTALL_TEMP_PATH);
        }
        uploadFinished = true;
        if (!uploadError) {
            showUploadProgress(UploadProgressStage::Write,
                               maxBytes, maxBytes);
        }
        MACPLUS_LOG("WEB: upload end, %lu bytes, %s\n",
               static_cast<unsigned long>(uploadOffset),
               uploadError ? "ERROR" : "OK");
        showTransferStatus(uploadError ? "[FAIL] WRITE ERROR"
                                       : "[OK] VERIFY COMPLETE");
    } else if (upload.status == RAW_ABORTED) {
        closeSdFile();
        if (target == UploadTarget::InstallDisk) remove(INSTALL_TEMP_PATH);
        uploadError = true;
        uploadFinished = false;
        MACPLUS_LOG("WEB: upload aborted at %lu bytes\n",
               static_cast<unsigned long>(uploadOffset));
        showTransferStatus("[FAIL] UPLOAD ABORTED");
    }
}

static void handleUploadData(UploadTarget target) {
    if (!webServer->header("Content-Type").startsWith(
            "application/octet-stream")) {
        uploadError = true;
        uploadFinished = true;
        uploadTarget = target;
        uploadOffset = 0;
        uploadExpectedBytes = 0;
        uploadRequestBytes = 0;
        uploadInputOffset = 0;
        showTransferStatus("[FAIL] UNSUPPORTED UPLOAD");
        return;
    }
    handleRawUpload(target);
}

static void handleHdUpload() { handleUploadData(UploadTarget::HardDisk); }
static void handleInstallUpload() { handleUploadData(UploadTarget::InstallDisk); }
static void handleHdUploadFinish() { handleUploadFinish(UploadTarget::HardDisk); }
static void handleInstallUploadFinish() {
    handleUploadFinish(UploadTarget::InstallDisk);
}

void webInstallRun() {
    MACPLUS_LOG("\n=== WiFi install mode ===\n");
    pinMode(MOUSE_BUTTON_PIN, INPUT_PULLUP);
    showTransferStatus("[RUN] STARTING AP");
    WiFi.mode(WIFI_AP);
    accessPointReady = WiFi.softAP("MacPlus-Install");
    accessPointAttempted = true;
    MACPLUS_LOG("WEB: open AP 'MacPlus-Install' (no password), then "
           "http://192.168.4.1\n");
    showTransferStatus(accessPointReady ? "[OK] READY FOR UPLOAD"
                                        : "[FAIL] AP START ERROR");

    webServer = accessPointReady ? new (std::nothrow) WebServer(80) : nullptr;
    if (accessPointReady && webServer == nullptr) {
        showTransferStatus("[FAIL] WEB SERVER MEMORY");
    } else if (webServer != nullptr) {
        const char *requestHeaders[] = {"Content-Length", "Content-Type"};
        webServer->collectHeaders(requestHeaders, 2);
        webServer->on("/", handleRoot);
        webServer->on("/info", handleInfo);
        webServer->on("/upload/hd", HTTP_POST, handleHdUploadFinish,
                      handleHdUpload);
        webServer->on("/upload/install", HTTP_POST, handleInstallUploadFinish,
                      handleInstallUpload);
        webServer->begin();
    }

    bool exitArmed = false;
    uint32_t exitHoldStartMs = 0;
    uint32_t lastKeyPollMs = 0;
    bool actionButtonDown = false;
    while (true) {
        if (webServer != nullptr) webServer->handleClient();
        const uint32_t now = millis();
        if (now - lastKeyPollMs >= 20U) {
            lastKeyPollMs = now;
            uint8_t key = cardputerInputReadKeyPress();
            while (key != 0xFFU) {
                handleWifiMenuKey(key);
                key = cardputerInputReadKeyPress();
            }
            const bool buttonDown = digitalRead(MOUSE_BUTTON_PIN) == LOW;
            if (buttonDown && !actionButtonDown) activateWifiMenu();
            actionButtonDown = buttonDown;
            const bool anyKey = cardputerInputAnyKeyPressed() ||
                                buttonDown;
            if (!exitArmed) {
                if (!anyKey) exitArmed = true;
            } else if (!anyKey) {
                exitHoldStartMs = 0;
            } else if (exitHoldStartMs == 0) {
                exitHoldStartMs = now;
            } else if (now - exitHoldStartMs >= 2000U) {
                showTransferStatus("[OK] EXITING TO MAC");
                exitWebInstallMode();
            }
        }
        delay(1);
    }
}
