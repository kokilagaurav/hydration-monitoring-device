const express = require('express');
const mongoose = require('mongoose');
const cors = require('cors');

const app = express();
app.use(express.json());
app.use(cors());

mongoose.connect('mongodb://127.0.0.1:27017/hydration_monitor', {
  useNewUrlParser: true,
  useUnifiedTopology: true
});

// User schema
const userSchema = new mongoose.Schema({
  username: { type: String, unique: true, required: true },
  password: { type: String, required: true },
  profile: {
    age: Number,
    weight: Number,
    activityLevel: String,
    hydrationGoal: Number // ml per day
  },
  hydrationHistory: [
    {
      timestamp: { type: Date, default: Date.now },
      status: String, // hydrated, mild, dehydrated
      gsr: Number,
      ir: Number,
      accel: Number,
      gyro: Number,
      mag: Number
    }
  ]
});

const User = mongoose.model('User', userSchema);

// Register
app.post('/api/register', async (req, res) => {
  const { username, password } = req.body;
  if (!username || !password) return res.status(400).json({ error: 'Missing fields' });
  try {
    const exists = await User.findOne({ username });
    if (exists) return res.status(409).json({ error: 'User exists' });
    const user = new User({ username, password });
    await user.save();
    res.json({ success: true });
  } catch (e) {
    res.status(500).json({ error: 'Registration failed' });
  }
});

// Login
app.post('/api/login', async (req, res) => {
  const { username, password } = req.body;
  const user = await User.findOne({ username });
  if (!user || user.password !== password) return res.status(401).json({ error: 'Invalid credentials' });
  res.json({ success: true });
});

// Save hydration data
app.post('/api/hydration', async (req, res) => {
  const { username, hydration } = req.body;
  if (!username || !hydration) return res.status(400).json({ error: 'Missing fields' });
  const user = await User.findOne({ username });
  if (!user) return res.status(404).json({ error: 'User not found' });
  user.hydrationHistory.push(hydration);
  await user.save();
  res.json({ success: true });
});

// Get hydration history & analytics
app.get('/api/hydration/:username', async (req, res) => {
  const user = await User.findOne({ username: req.params.username });
  if (!user) return res.status(404).json({ error: 'User not found' });
  // Calculate streaks and achievements
  let streak = 0;
  for (let i = user.hydrationHistory.length - 1; i >= 0; i--) {
    if (user.hydrationHistory[i].status === 'hydrated') streak++;
    else break;
  }
  const hydratedCount = user.hydrationHistory.filter(h => h.status === 'hydrated').length;
  const achievements = [];
  if (streak >= 3) achievements.push('3-day Streak');
  if (hydratedCount >= 10) achievements.push('10x Hydrated');
  res.json({
    hydrationHistory: user.hydrationHistory,
    streak,
    achievements
  });
});

// Update user profile
app.post('/api/profile', async (req, res) => {
  const { username, profile } = req.body;
  const user = await User.findOne({ username });
  if (!user) return res.status(404).json({ error: 'User not found' });
  user.profile = profile;
  await user.save();
  res.json({ success: true });
});

// Export hydration data as CSV
app.get('/api/export/:username', async (req, res) => {
  const user = await User.findOne({ username: req.params.username });
  if (!user) return res.status(404).json({ error: 'User not found' });
  let csv = 'timestamp,status,gsr,ir,accel,gyro,mag\n';
  csv += user.hydrationHistory.map(h => `${h.timestamp.toISOString()},${h.status},${h.gsr},${h.ir},${h.accel},${h.gyro},${h.mag}`).join('\n');
  res.header('Content-Type', 'text/csv');
  res.attachment('hydration_history.csv');
  res.send(csv);
});

// Share hydration report (returns summary)
app.get('/api/share/:username', async (req, res) => {
  const user = await User.findOne({ username: req.params.username });
  if (!user) return res.status(404).json({ error: 'User not found' });
  const last = user.hydrationHistory[user.hydrationHistory.length - 1];
  res.json({
    username: user.username,
    lastStatus: last ? last.status : 'No data',
    streak: user.hydrationHistory.length,
    achievements: user.hydrationHistory.length
  });
});

app.listen(4000, () => {
  console.log('Hydration backend running on http://localhost:4000');
});