'use strict';
(() => {
  const flasher = document.querySelector('esp-web-install-button');
  if (!flasher) return;
  const button = document.createElement('button');
  button.type = 'button';
  button.className = 'serial-button';
  button.textContent = 'ボタンを押さずに書き込み準備';
  const message = document.createElement('p');
  message.textContent = '対応版を書き込み済みの場合、USB接続し停止中にこのボタンを押してから、上の書き込みボタンを押します。';
  flasher.insertAdjacentElement('afterend', button);
  button.insertAdjacentElement('afterend', message);
  button.addEventListener('click', async () => {
    let port, reader, writer;
    try {
      if (!navigator.serial) throw new Error('Chrome または Edge が必要です。');
      port = await navigator.serial.requestPort();
      await port.open({baudRate: 115200});
      reader = port.readable.getReader();
      writer = port.writable.getWriter();
      await writer.write(new TextEncoder().encode('ENTER DOWNLOAD\n'));
      writer.releaseLock(); writer = null;
      let data = '';
      const timeout = setTimeout(() => { if (reader) void reader.cancel(); }, 2500);
      try {
        while (!/USBDBG,download=(starting|busy|failed)/.test(data)) {
          const result = await reader.read();
          if (result.done) break;
          data += new TextDecoder().decode(result.value);
        }
      } finally { clearTimeout(timeout); }
      if (data.includes('download=busy')) message.textContent = '動作中は書き込み準備できません。停止してから再試行してください。';
      else if (data.includes('download=starting')) message.textContent = '書き込み準備を開始しました。数秒待って上の書き込みボタンを押し、再表示されたUSBポートを選んでください。';
      else message.textContent = '応答を確認できません。対応版か、USBログ接続が切断されているか確認してください。';
    } catch (error) { message.textContent = `書き込み準備に失敗: ${error.message}`; }
    finally {
      if (writer) writer.releaseLock();
      if (reader) { try { await reader.cancel(); } catch (_) {} reader.releaseLock(); }
      if (port) { try { await port.close(); } catch (_) {} }
    }
  });
})();

