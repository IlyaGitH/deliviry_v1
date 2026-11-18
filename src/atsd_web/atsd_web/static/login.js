const fragment = new URLSearchParams(location.hash.slice(1));
const requestedRole = fragment.get('role') || new URLSearchParams(location.search).get('role');
const role = ['sender','receiver'].includes(requestedRole) ? requestedRole : 'admin';
const order = fragment.get('order');
const token = fragment.get('token');
history.replaceState(null, '', '/login');
if (role !== 'admin') {
  document.getElementById('purpose').textContent = role === 'sender' ? 'Доступ отправителя к заказу' : 'Доступ получателя к заказу';
  const field = document.getElementById('secret');
  field.required = false; field.disabled = true; field.hidden = true;
  document.getElementById('secretLabel').hidden = true;
  const button = document.querySelector('button[type=submit]');
  button.textContent = role === 'sender' ? 'Войти как отправитель' : 'Войти как получатель';
  if (!order || !token || !/^[A-Za-z0-9_-]{20,256}$/.test(token)) {
    button.disabled = true;
    document.getElementById('result').textContent = 'Ссылка неполная. Откройте исходную персональную ссылку целиком, включая часть после #. Если она потеряна, администратор может выдать новую.';
  } else {
    document.getElementById('purpose').textContent += '. Пароль не нужен — доступ содержится в ссылке.';
  }
}
document.getElementById('loginForm').onsubmit = async event => {
  event.preventDefault();
  const result = document.getElementById('result');
  const button = event.target.querySelector('button');
  button.disabled = true;
  try {
    const response = await fetch('/api/security/login', {method:'POST',
      headers:{'Content-Type':'application/json', 'X-ATSD-Request':'1'},
      body:JSON.stringify({role,order,secret:role === 'admin' ? document.getElementById('secret').value : token})});
    const data = await response.json();
    if (!response.ok) throw new Error(data.detail || 'Вход не выполнен');
    location.replace('/');
  } catch(error) { result.textContent = error.message; }
  finally { button.disabled = false; }
};
