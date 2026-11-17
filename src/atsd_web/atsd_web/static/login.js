const fragment = new URLSearchParams(location.hash.slice(1));
const role = ['sender','receiver'].includes(fragment.get('role')) ? fragment.get('role') : 'admin';
const order = fragment.get('order');
const token = fragment.get('token');
history.replaceState(null, '', '/login');
if (role !== 'admin') {
  document.getElementById('purpose').textContent = role === 'sender' ? 'Доступ отправителя к заказу' : 'Доступ получателя к заказу';
  if (token) document.getElementById('secret').value = token;
}
document.getElementById('loginForm').onsubmit = async event => {
  event.preventDefault();
  const result = document.getElementById('result');
  const button = event.target.querySelector('button');
  button.disabled = true;
  try {
    const response = await fetch('/api/security/login', {method:'POST',
      headers:{'Content-Type':'application/json', 'X-ATSD-Request':'1'},
      body:JSON.stringify({role,order,secret:document.getElementById('secret').value})});
    const data = await response.json();
    if (!response.ok) throw new Error(data.detail || 'Вход не выполнен');
    location.replace('/');
  } catch(error) { result.textContent = error.message; }
  finally { button.disabled = false; }
};
